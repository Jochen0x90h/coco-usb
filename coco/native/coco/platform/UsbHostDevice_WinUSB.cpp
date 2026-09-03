#include <coco/platform/WindowsDef.hpp>
#include <windows.h>
#include <winusb.h>
#include <SetupAPI.h>
#include <initguid.h>
#include <usbiodef.h>
#include <Dbt.h>
#include <coco/platform/WindowsUndef.hpp>

#include "UsbHostDevice_WinUSB.hpp"
#include <coco/convert.hpp>


namespace coco {

UsbHostDevice_WinUSB::UsbHostDevice_WinUSB(Loop_Win32 &loop)
    : Device(State::DISABLED)
    , loop_(loop)
{
    loop.addDeviceHandler(*this);
}

UsbHostDevice_WinUSB::~UsbHostDevice_WinUSB() {
    close();
}

bool UsbHostDevice_WinUSB::open(DevicePath path) {
    if (handle_ != INVALID_HANDLE_VALUE)
        return false;

    // open the device
    auto handle = CreateFileW(path.c_str(),
        GENERIC_READ | GENERIC_WRITE,
        0, // no sharing
        nullptr, // no security
        OPEN_EXISTING, // open only existing device
        FILE_FLAG_OVERLAPPED,
        nullptr);
    if (handle == INVALID_HANDLE_VALUE) {
        int error = GetLastError();
        setSystemError(error);
        return false;
    }

    // get USB interface
    WINUSB_INTERFACE_HANDLE interface;
    if (!WinUsb_Initialize(handle, &interface)) {
        int error = GetLastError();
        setSystemError(error);
        CloseHandle(handle);
        return false;
    }

    // add handle to completion port of event loop
    CreateIoCompletionPort(
        handle,
        loop_.port(),
        ULONG_PTR(&static_cast<Loop_Win32::CompletionHandler &>(*this)),
        0);

    path_ = path.path();
    handle_ = handle;
    interface_ = interface;
    setSuccess();

    USB_INTERFACE_DESCRIPTOR interfaceDescriptor;
    auto result = WinUsb_QueryInterfaceSettings(interface, 0, &interfaceDescriptor);

    // set state
    state_ = State::READY;

    // enable buffers
    for (auto &buffer : controlBuffers_) {
        buffer.setReady();
    }
    for (auto &endpoint : endpoints_) {
        endpoint.state_ = State::READY;
        for (auto &buffer : endpoint.buffers_) {
            buffer.setReady();
        }

        // resume all coroutines waiting for state change
        endpoint.notify(Events::ENTER_READY);
    }

    // resume all coroutines waiting for state change
    notify(Events::ENTER_READY);

    return true;
}

void UsbHostDevice_WinUSB::close() {
    if (handle_ == INVALID_HANDLE_VALUE)
        return;

    // close interface
    WinUsb_Free(interface_);
    interface_ = nullptr;

    // close handle
    CloseHandle(handle_);
    handle_ = INVALID_HANDLE_VALUE;

    // clear path
    path_.clear();
    setSuccess();

    // set state
    state_ = State::DISABLED;

    // disable buffers
    for (auto &buffer : controlBuffers_) {
        buffer.setDisabled();
    }
    for (auto &endpoint : endpoints_) {
        endpoint.state_ = State::DISABLED;
        for (auto &buffer : endpoint.buffers_) {
            buffer.setDisabled();
        }

        // resume all coroutines waiting for state change
        endpoint.notify(Events::ENTER_CLOSING | Events::ENTER_DISABLED);
    }

    // resume all coroutines waiting for state change
    notify(Events::ENTER_CLOSING | Events::ENTER_DISABLED);
}

/*
void UsbHost_WinUSB::Device::getDescriptor(usb::DescriptorType type, void *data, int &size) {
    int index = 0;
    int languageId = 0;
      ULONG transferred;
    bool result = WinUsb_GetDescriptor(interface, int(type), index, languageId, (UCHAR*)data, size, &transferred);
    size = result ? transferred : 0;
}*/

/*
static bool getIsoPipeInfo(void *interface, int endpointCount, uint8_t endpointAddress, WINUSB_PIPE_INFORMATION_EX &pipeInfo) {
    for (int i = 0; i < endpointCount; ++i) {
        auto result = WinUsb_QueryPipeEx(interface, 0, i, &pipeInfo);

        if (result && pipeInfo.PipeType == UsbdPipeTypeIsochronous && pipeInfo.PipeId == endpointAddress)
            return true;
    }
    return false;
}*/

void UsbHostDevice_WinUSB::onCompletion(OVERLAPPED *overlapped) {
    for (auto &buffer : controlBuffers_) {
        if (overlapped == &buffer.overlapped_) {
            buffer.onCompletion(overlapped);
            return;
        }
    }
    for (auto &endpoint : endpoints_) {
        for (auto &buffer : endpoint.buffers_) {
            if (overlapped == &buffer.overlapped_[buffer.index_]) {
                buffer.onCompletion(overlapped);
                return;
            }
        }
    }
}

void UsbHostDevice_WinUSB::onDeviceChange(Loop_Win32::DeviceType type, bool add, DevicePath path) {
    if (type != Loop_Win32::DeviceType::USB || add)
        return;

    if (path_ == path.c_str()) {
        // USB device was removed
        close();
    }
}


// ControlBuffer

UsbHostDevice_WinUSB::ControlBuffer::ControlBuffer(int capacity, UsbHostDevice_WinUSB &device)
    : Buffer(&setup_, sizeof(setup_), new uint8_t[capacity], capacity, device.state_)
    , device_(device)
{
    device.controlBuffers_.add(*this);
}

UsbHostDevice_WinUSB::ControlBuffer::~ControlBuffer() {
    delete [] data_;
}

bool UsbHostDevice_WinUSB::ControlBuffer::start() {
    // note the transfer size is 0 for control transfers without data stage
    if (state_ != State::READY) {
        assert(false);
        setError(std::errc::resource_unavailable_try_again);
        return false;
    }
    if ((op_ & Op::READ_WRITE) == 0) {
        setSuccess();
        return false;
    }

    steps_ = 1;

    // start transfer
    if (!transfer())
        return false;

    // set state
    setBusy();

    return true;
}

bool UsbHostDevice_WinUSB::ControlBuffer::cancel() {
    if (state_ != State::BUSY)
        return false;

    if (steps_ != 0) {
        // cancel the transfer, the io completion port will receive ERROR_OPERATION_ABORTED
        auto result = CancelIoEx(device_.handle_, &overlapped_);
        if (!result) {
            int error = GetLastError();
            setSystemError(error);
            return false;
        }
        steps_ = 0;
    }

    return true;
}

bool UsbHostDevice_WinUSB::ControlBuffer::transfer() {
    DWORD transferred;
    memset(&overlapped_, 0, sizeof(OVERLAPPED));
    auto result = WinUsb_ControlTransfer(device_.interface_, setup_, data_, size_, &transferred, &overlapped_);
    Sleep(10);
    if (result != 0) {
        setSuccess(transferred);
    } else {
        int error = GetLastError();
        if (error == ERROR_IO_PENDING) {
            // wait for completion
            return true;
        }
        setSystemError(error);
    }
    return false;
}

void UsbHostDevice_WinUSB::ControlBuffer::onCompletion(OVERLAPPED *overlapped) {
    DWORD transferred;
    auto result = GetOverlappedResult(device_.handle_, overlapped, &transferred, false);
    if (result) {
        // success
        setSuccess(transferred);
    } else {
        // error
        // canceled: ERROR_OPERATION_ABORTED
        auto error = GetLastError();
        setSystemError(error);

        // check if the USB device was disconnected
        if (error == ERROR_GEN_FAILURE)
            device_.close();//disconnect();
    }

    // transfer finished
    setReady();
}


// Buffer

UsbHostDevice_WinUSB::Buffer::Buffer(int capacity, Endpoint &endpoint)
    : coco::Buffer(new uint8_t[capacity], capacity, endpoint.device_.state_)
    , endpoint_(endpoint)
{
    endpoint.buffers_.add(*this);
}

UsbHostDevice_WinUSB::Buffer::~Buffer() {
    delete [] data_;
}

bool UsbHostDevice_WinUSB::Buffer::start() {
    if (state_ != State::READY) {
        assert(false);
        setError(std::errc::resource_unavailable_try_again);
        return false;
    }
    if ((op_ & Op::READ_WRITE) == 0 || size_ == 0) {
        setSuccess();
        return false;
    }

    auto &device = endpoint_.device_;

    steps_ = int(op_ & Op::READ_WRITE);

    // start transfer
    if (!transfer())
        return false;

    // set state
    setBusy();

    return true;
}

bool UsbHostDevice_WinUSB::Buffer::cancel() {
    if (state_ != State::BUSY)
        return false;

    if (steps_ != 0) {
        // cancel the transfer, the io completion port will receive ERROR_OPERATION_ABORTED
        for (int i = index_; i >= 0; --i) {
            auto result = CancelIoEx(endpoint_.device_.handle_, &overlapped_[i]);
            if (!result) {
                int error = GetLastError();
                setSystemError(error);
                return false;
            }
        }
        steps_ = 0;
    }

    return true;
}

bool UsbHostDevice_WinUSB::Buffer::transfer() {
    auto &device = endpoint_.device_;
    memset(&overlapped_[0], 0, sizeof(OVERLAPPED));
    int index = 0;
    auto data = data_;
    if ((Op(steps_) & Op::WRITE) == 0) {
        // read
        //op_ = Op::NONE;
        int size = capacity_;
        auto result = WinUsb_ReadPipe(device.interface_, endpoint_.inAddress_, data, size,
            nullptr, &overlapped_[0]);
    } else {
        // write
        int size = size_;

        // check if we need to send a zero packet at the end
        bool zero = (op_ & Op::PARTIAL) == 0 && size > 0 && (size & 63) == 0;
        if (zero) {
            memset(&overlapped_[1], 0, sizeof(OVERLAPPED));
            index = 1;
        }

        auto result = WinUsb_WritePipe(device.interface_, endpoint_.outAddress_, data, size,
            nullptr, &overlapped_[index]);

        // send a zero packet at the end if necessary
        if (zero) {
            result = WinUsb_WritePipe(device.interface_, endpoint_.outAddress_, data + size, 0,
                nullptr, &overlapped_[0]);
        }
    }
    index_ = index;
    size_ = 0;
    return true;
}

void UsbHostDevice_WinUSB::Buffer::onCompletion(OVERLAPPED *overlapped) {
    auto &device = endpoint_.device_;
    DWORD transferred;
    auto result = GetOverlappedResult(device.handle_, overlapped, &transferred, false);
    if (result) {
        // transfer OK
        if (index_ > 0) {
            // need to wait for completion of the zero packet
            index_ = 0;
            size_ = transferred;
            return;
        }
        if (Op(steps_) == Op::READ_WRITE) {
            // read after write
            steps_ = int(Op::READ);
            transfer();
            return;
        }

        // transfer finished
        setSuccess(size_ + transferred);
    } else {
        // error
        // canceled: ERROR_OPERATION_ABORTED
        auto error = GetLastError();
        setSystemError(error);

        // check if the USB device was disconnected
        if (error == ERROR_GEN_FAILURE)
            device.close();
    }

    // transfer finished
    setReady();
}


// Endpoint

UsbHostDevice_WinUSB::Endpoint::Endpoint(UsbHostDevice_WinUSB &device, int inAddress, int outAddress)
    : BufferDevice(State::OPENING)
    , device_(device), inAddress_(inAddress), outAddress_(outAddress)
{
    device.endpoints_.add(*this);
}

UsbHostDevice_WinUSB::Endpoint::~Endpoint() {
}

int UsbHostDevice_WinUSB::Endpoint::getBufferCount() {
    return buffers_.count();
}

UsbHostDevice_WinUSB::Buffer &UsbHostDevice_WinUSB::Endpoint::getBuffer(int index) {
    return buffers_.get(index);
}

} // namespace coco

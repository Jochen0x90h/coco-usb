#include "UsbHost_WinUSB.hpp" // includes Windows.h and winusb.h
#include <coco/convert.hpp>
#include <iostream>
#include <SetupAPI.h>
#include <initguid.h>
#include <usbiodef.h>


namespace coco {

UsbHost_WinUSB::UsbHost_WinUSB(Loop_Win32 &loop)
    : loop_(loop)
    //, callback_(makeCallback<UsbHost_WinUSB, &UsbHost_WinUSB::onTimeout>(this))
{
    // regularly scan for usb devices
    loop.invoke(*this);
}

//UsbHost_WinUSB::~UsbHost_WinUSB() {
//}
/*
static int parseHex(const char *s) {
    int value = 0;
    for (int i = 0; i < 4; ++i) {
        value <<= 4;

        char ch = s[i];
        if (ch >= '0' && ch <= '9')
            value += ch - '0';
        else if (ch >= 'A' && ch <= 'F')
            value += ch - 'A' + 10;
        else if (ch >= 'a' && ch <= 'f')
            value += ch - 'a' + 10;
    }
    return value;
}*/

void UsbHost_WinUSB::onTimeout() {
    loop_.invoke(*this, 1s);

    // enumerate devices
    HDEVINFO deviceInfo = SetupDiGetClassDevsA(nullptr, nullptr, nullptr, DIGCF_ALLCLASSES | DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);
    int index = 0;
    SP_DEVINFO_DATA deviceData;
    deviceData.cbSize = sizeof(SP_DEVINFO_DATA);
    while (SetupDiEnumDeviceInfo(deviceInfo, index, &deviceData)) {
        ++index;

        // get interface data
        SP_DEVICE_INTERFACE_DATA interfaceData;
        interfaceData.cbSize = sizeof(SP_DEVICE_INTERFACE_DATA);
        if (!SetupDiEnumDeviceInterfaces(deviceInfo, &deviceData, &GUID_DEVINTERFACE_USB_DEVICE, 0, &interfaceData)) {
            continue;
        }

        // get device path
        union {
            SP_DEVICE_INTERFACE_DETAIL_DATA_A devicePath;
            uint8_t space[2 + 128];
        } u;
        u.devicePath.cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA_A);
        if (!SetupDiGetDeviceInterfaceDetailA(deviceInfo, &interfaceData, &u.devicePath, sizeof(u), nullptr, nullptr)) {
            // error
            continue;
        }

        // check if device is new
        auto p = deviceInfos_.insert(std::map<std::string, Device *>::value_type{u.devicePath.DevicePath, nullptr});
        auto it = p.first;
        if (p.second) {
            // found a new device, path has the form \\?\usb#vid_1915&pid_1337#5&41045ef&0&4#{a5dcbf10-6530-11d2-901f-00c04fb951ed}

            // try to open the device
            auto file = CreateFileA(it->first.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, FILE_FLAG_OVERLAPPED, nullptr);
            if (file == INVALID_HANDLE_VALUE)
                continue;

            // get USB interface
            WINUSB_INTERFACE_HANDLE interface;
            if (!WinUsb_Initialize(file, &interface)) {
                CloseHandle(file);
                continue;
            }

            // read device descriptor
            usb::DeviceDescriptor deviceDescriptor;
              ULONG transferred;
            bool result = WinUsb_GetDescriptor(interface, int(usb::DescriptorType::DEVICE), 0, 0, (UCHAR*)&deviceDescriptor, sizeof(deviceDescriptor), &transferred);
            if (!result || transferred < sizeof(deviceDescriptor)) {
                WinUsb_Free(interface);
                CloseHandle(file);
                continue;
            }

            // now check if a filter of a device accepts the device descriptor
            for (Device &device : devices_) {
                if (device.state_ == Device::State::OPENING && device.filter_(deviceDescriptor)) {
                    // add file to completion port of event loop
                    CreateIoCompletionPort(
                        file,
                        loop_.port,
                        ULONG_PTR(&static_cast<Loop_Win32::CompletionHandler &>(device)),
                        0);

                    // set device to map and store the iterator to be able to erase the device in disconnect()
                    it->second = &device;
                    device.it_ = it;

                    // connect and transfer ownership of file to device
                    device.connect(file, interface);
                    file = INVALID_HANDLE_VALUE;

                    // set iterator and flag the device to indicate that it is present
                    device.flag_ = true;
                    break;
                }
            }

            // device was not accepted by any filter, close file and free interface
            if (file != INVALID_HANDLE_VALUE) {
                WinUsb_Free(interface);
                CloseHandle(file);
            }
        } else {
            // flag the device to indicate that it is still present
            if (it->second != nullptr)
                it->second->flag_ = true;
        }
    }

    // detect devices that have disappeared
    for (Device &device : devices_) {
        if (device.ready() && !device.flag_)
            device.disconnect();
        device.flag_ = false;
    }
}


// Device

UsbHost_WinUSB::Device::Device(UsbHost_WinUSB &host, std::function<bool (const usb::DeviceDescriptor &)> filter)
    : UsbHostDevice(State::DISABLED), host_(host), filter_(filter)
{
    host.devices_.add(*this);
}

UsbHost_WinUSB::Device::~Device() {
    WinUsb_Free(interface_);
    CloseHandle(file_);
}

void UsbHost_WinUSB::Device::open() {
    if (state_ == State::DISABLED) {
        state_ = State::OPENING;
        notify(Events::ENTER_OPENING);
    }
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

void UsbHost_WinUSB::Device::connect(HANDLE file, void *interface) {
    file_ = file;
    interface_ = interface;


    USB_INTERFACE_DESCRIPTOR interfaceDescriptor;
    auto result = WinUsb_QueryInterfaceSettings(interface, 0, &interfaceDescriptor);
/*
    // allocate buffers of isochronous endpoints
    for (auto &endpoint : isoEndpoints) {
        WINUSB_PIPE_INFORMATION_EX pipeInfo;
        if ((endpoint.inAddress & 0x7f) != 0) {
            if (getIsoPipeInfo(interface, interfaceDescriptor.bNumEndpoints, endpoint.inAddress, pipeInfo)) {
                int packetSize = pipeInfo.MaximumBytesPerInterval;
                int packetCount = (endpoint.duration * (8 / pipeInfo.Interval) + 1) / 3;
                int bufferSize = packetCount * packetSize;

                uint8_t *buffer = new uint8_t[bufferSize * 3];

                WINUSB_ISOCH_BUFFER_HANDLE bufferHandle;
                result = WinUsb_RegisterIsochBuffer(
                    interface,
                    endpoint.inAddress,
                    buffer,
                    bufferSize * 3,
                    &bufferHandle);

                endpoint.inPacketSize = packetSize;
                endpoint.inPacketCount = packetCount;
                endpoint.inBufferSize = bufferSize;
                endpoint.inBuffer = buffer;
                endpoint.inBufferHandle = bufferHandle;
                endpoint.inPacketDescriptors = new USBD_ISO_PACKET_DESCRIPTOR[packetCount * 3];

            }
        }
    }
*/

    // set state of device and buffers to READY
    state_ = State::READY;
    for (auto &buffer : controlBuffers_) {
        buffer.setReady();
    }
    for (auto &endpoint : endpoints_) {
        endpoint.state_ = State::READY;
        for (auto &buffer : endpoint.buffers_) {
            buffer.setReady();
        }

        // resume coroutines waiting for state change
        endpoint.notify(Events::ENTER_READY);
    }

    // resume coroutines waiting for state change
    notify(Events::ENTER_READY);
}

void UsbHost_WinUSB::Device::disconnect() {
    if (state_ == Device::State::READY) {
        flag_ = false;
        WinUsb_Free(interface_);
        CloseHandle(file_);
        interface_ = nullptr;
        file_ = INVALID_HANDLE_VALUE;

        // erase from device map of host
        host_.deviceInfos_.erase(it_);

        // set state of device to CLOSING and state of buffers to DISABLED
        state_ = State::DISABLED;
        for (auto &buffer : controlBuffers_) {
            buffer.setDisabled();
        }
        for (auto &endpoint : endpoints_) {
            endpoint.state_ = State::DISABLED;
            for (auto &buffer : endpoint.buffers_) {
                buffer.setDisabled();
            }

            // resume coroutines waiting for state change
            endpoint.notify(Events::ENTER_CLOSING | Events::ENTER_DISABLED);
        }

        // resume coroutines waiting for state change
        notify(Events::ENTER_CLOSING | Events::ENTER_DISABLED);
    }
}

void UsbHost_WinUSB::Device::onCompletion(OVERLAPPED *overlapped) {
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
    /*for (auto &buffer : controlTransfers_) {
        if (overlapped == &buffer.overlapped_) {
            buffer.handle(overlapped);
            return;
        }
    }
    for (auto &buffer : transfers_) {
        if (overlapped == &buffer.overlapped_[buffer.index_]) {
            buffer.handle(overlapped);
            return;
        }
    }*/
}


// ControlBuffer

UsbHost_WinUSB::ControlBuffer::ControlBuffer(int capacity, Device &device)
    : Buffer(&setup_, sizeof(WINUSB_SETUP_PACKET), new uint8_t[capacity], capacity, device.state_)
    , device_(device)
{
    device.controlBuffers_.add(*this);
}

UsbHost_WinUSB::ControlBuffer::~ControlBuffer() {
    delete [] data_;
}

bool UsbHost_WinUSB::ControlBuffer::start() {
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

    // add to list of pending transfers
    //device_.controlTransfers_.add(*this);

    // start transfer
    if (!transfer())
        return false;

    // set state
    setBusy();

    return true;
}

bool UsbHost_WinUSB::ControlBuffer::cancel() {
    if (state_ != State::BUSY)
        return false;

    if (steps_ != 0) {
        // cancel the transfer, the io completion port will receive ERROR_OPERATION_ABORTED
        auto result = CancelIoEx(device_.file_, &overlapped_);
        if (!result) {
            int error = GetLastError();
            setSystemError(error);
            return false;
        }
        steps_ = 0;
    }

    return true;
}

bool UsbHost_WinUSB::ControlBuffer::transfer() {
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

void UsbHost_WinUSB::ControlBuffer::onCompletion(OVERLAPPED *overlapped) {
    DWORD transferred;
    auto result = GetOverlappedResult(device_.file_, overlapped, &transferred, false);
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
            device_.disconnect();
    }

    // remove from list of active transfers
    //remove2();

    // transfer finished
    setReady();
}


// Buffer

UsbHost_WinUSB::Buffer::Buffer(int capacity, Endpoint &endpoint)
    : coco::Buffer(new uint8_t[capacity], capacity, endpoint.device_.state_)
    , endpoint_(endpoint)
{
    endpoint.buffers_.add(*this);
}

UsbHost_WinUSB::Buffer::~Buffer() {
    delete [] data_;
}

bool UsbHost_WinUSB::Buffer::start() {
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

    // add to list of pending transfers
    //device.transfers_.add(*this);

    steps_ = int(op_ & Op::READ_WRITE);

    // start transfer
    if (!transfer())
        return false;

    // set state
    setBusy();

    return true;
}

bool UsbHost_WinUSB::Buffer::cancel() {
    if (state_ != State::BUSY)
        return false;

    if (steps_ != 0) {
        // cancel the transfer, the io completion port will receive ERROR_OPERATION_ABORTED
        for (int i = index_; i >= 0; --i) {
            auto result = CancelIoEx(endpoint_.device_.file_, &overlapped_[i]);
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

bool UsbHost_WinUSB::Buffer::transfer() {
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

void UsbHost_WinUSB::Buffer::onCompletion(OVERLAPPED *overlapped) {
    auto &device = endpoint_.device_;
    DWORD transferred;
    auto result = GetOverlappedResult(device.file_, overlapped, &transferred, false);
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
            device.disconnect();
    }

    // remove from list of active transfers
    //remove2();

    // transfer finished
    setReady();
}


// Endpoint

UsbHost_WinUSB::Endpoint::Endpoint(UsbHost_WinUSB::Device &device, int inAddress, int outAddress)
    : BufferDevice(State::OPENING)
    , device_(device), inAddress_(inAddress), outAddress_(outAddress)
{
    device.endpoints_.add(*this);
}

UsbHost_WinUSB::Endpoint::~Endpoint() {
}

int UsbHost_WinUSB::Endpoint::getBufferCount() {
    return buffers_.count();
}

UsbHost_WinUSB::Buffer &UsbHost_WinUSB::Endpoint::getBuffer(int index) {
    return buffers_.get(index);
}

} // namespace coco

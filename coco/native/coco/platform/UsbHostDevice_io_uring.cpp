#include "UsbHostDevice_io_uring.hpp"
#include <coco/convert.hpp>
#include <coco/platform/NativeFile.hpp>
#include <iostream>
#include <cstring>
#include <sys/ioctl.h>
//#include <linux/usb/ch9.h> // struct usb_device_descriptor


namespace coco {

// Device

UsbHostDevice_io_uring::UsbHostDevice_io_uring(Loop_io_uring &loop)
    : Device(State::DISABLED)
    , loop_(loop)
{
}

UsbHostDevice_io_uring::~UsbHostDevice_io_uring() {
    close();
}

bool UsbHostDevice_io_uring::open(const std::filesystem::path &path) {
    if (handle_ != INVALID_HANDLE_VALUE)
        return false;

    // open the device
    int handle = ::open(path.c_str(), O_RDWR);
    if (handle == INVALID_HANDLE_VALUE) {
        int error = errno;
        setSystemError(error);
        return false;
    }

    handle_ = handle;
    setSuccess();

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

    // poll read events for urb completions and disconnect
    loop_.poll(*this, handle_, POLLIN | POLLOUT);
    return true;
}

void UsbHostDevice_io_uring::close() {
    if (handle_ == INVALID_HANDLE_VALUE)
        return;

    // close handle
    ::close(handle_);
    handle_ = INVALID_HANDLE_VALUE;

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

void UsbHostDevice_io_uring::onCompletion(io_uring_cqe &cqe, int id) {
    if (cqe.res & (POLLERR | POLLHUP | POLLNVAL)) {
        // device disconnected
        close();
    } else if (cqe.res & (POLLIN | POLLOUT)) {
        // poll again
        loop_.poll(*this, handle_, POLLIN | POLLOUT);

        // get completed urbs
        while (true) {
            struct usbdevfs_urb* urb = nullptr;
            int result = ioctl(handle_, USBDEVFS_REAPURBNDELAY, &urb);

            if (result < 0) {
                break;
            }

            if (urb) {
                Buffer *buffer = static_cast<Buffer *>(urb->usercontext);
                buffer->onCompletion();
            }
        }
    }
}


// ControlBuffer

UsbHostDevice_io_uring::ControlBuffer::ControlBuffer(int capacity, UsbHostDevice_io_uring &device)
    : Buffer(&setup_, sizeof(setup_), new uint8_t[capacity], capacity, device.state_)
    , device_(device)
{
    device.controlBuffers_.add(*this);
}

UsbHostDevice_io_uring::ControlBuffer::~ControlBuffer() {
    delete [] data_;
}

bool UsbHostDevice_io_uring::ControlBuffer::start() {
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

bool UsbHostDevice_io_uring::ControlBuffer::cancel() {
    if (state_ != State::BUSY)
        return false;

    if (steps_ != 0) {
        // cancel the ioctl
        /*auto result = CancelIoEx(device_.file_, &overlapped_);
        if (!result) {
            int error = GetLastError();
            setSystemError(error);
            return false;
        }*/
        steps_ = 0;
    }

    return true;
}

bool UsbHostDevice_io_uring::ControlBuffer::transfer() {
    struct usbdevfs_ctrltransfer ctrl;
    reinterpret_cast<usb::Setup &>(ctrl) = setup_;
    ctrl.timeout = 100;
    ctrl.data = data_;

    int transferred = ioctl(device_.handle_, USBDEVFS_CONTROL, &ctrl);

    if (transferred >= 0) {
        setSuccess(transferred);
    } else {
        int error = errno;
        setSystemError(error);
    }

    return false;
}


// Buffer

UsbHostDevice_io_uring::Buffer::Buffer(int capacity, Endpoint &endpoint)
    : coco::Buffer(new uint8_t[capacity], capacity, endpoint.device_.state_)
    , endpoint_(endpoint)
{
    endpoint.buffers_.add(*this);

    int urbSize = sizeof(struct usbdevfs_urb) * 2;
    urb_ = static_cast<struct usbdevfs_urb*>(std::malloc(urbSize));
    urb0_ = urb_ + 1;
    memset(urb_, 0, urbSize);
}

UsbHostDevice_io_uring::Buffer::~Buffer() {
    delete [] data_;
    std::free(urb_);
}

bool UsbHostDevice_io_uring::Buffer::start() {
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

bool UsbHostDevice_io_uring::Buffer::cancel() {
    if (state_ != State::BUSY)
        return false;

    if (steps_ != 0) {
        // cancel the transfer, the io completion port will receive ERROR_OPERATION_ABORTED
        int result = ioctl(endpoint_.device_.handle_, USBDEVFS_DISCARDURB, urb_);
        if (result < 0) {
            int error = errno;
            setSystemError(error);
            return false;
        }
        steps_ = 0;

        // get completed urbs (linux bug: POLLIN does not trigger)
        while (true) {
            struct usbdevfs_urb* urb = nullptr;
            int result = ioctl(endpoint_.device_.handle_, USBDEVFS_REAPURBNDELAY, &urb);

            if (result < 0) {
                break;
            }

            if (urb) {
                Buffer *buffer = static_cast<Buffer *>(urb->usercontext);
                buffer->onCompletion();
            }
        }
    }

    return true;
}

bool UsbHostDevice_io_uring::Buffer::transfer() {
    auto &endpoint = endpoint_;
    auto &device = endpoint.device_;
    auto &urb = *urb_;

    urb.type = USBDEVFS_URB_TYPE_BULK;
    urb.buffer = data_;
    urb.buffer_length = size_;
    urb.usercontext = this;

    bool read = (Op(steps_) & Op::WRITE) == 0;
    if (read) {
        // read
        urb.endpoint = endpoint.inAddress_;
    } else {
        // write
        urb.endpoint = endpoint.outAddress_;
    }

    urb.status = 0;
    urb.actual_length = 0;
    urb.error_count = 0;
    int result = ioctl(device.handle_, USBDEVFS_SUBMITURB, &urb);

    if (!read && (size_ & 63) == 0) {
        // send ZLP
        steps_ |= 0x80; // mark ZLP step
        auto &urb = *urb0_;

        urb.type = USBDEVFS_URB_TYPE_BULK;
        urb.buffer = data_;
        urb.buffer_length = 0;
        urb.usercontext = this;

        // write
        urb.endpoint = endpoint.outAddress_;

        urb.status = 0;
        urb.actual_length = 0;
        urb.error_count = 0;
        result = ioctl(device.handle_, USBDEVFS_SUBMITURB, &urb);
    }

    return true;
}

void UsbHostDevice_io_uring::Buffer::onCompletion() {
    auto &urb = *urb_;
    int status = urb.status;
    if (status == 0) {
        // transfer OK
        if (Op(steps_) == Op::READ_WRITE) {
            // read after write
            size_ = capacity_;
            steps_ = int(Op::READ);
            transfer();
            return;
        }

        if ((steps_ & int(Op::WRITE)) != 0) {
            if (steps_ & 0x80) {
                // ZLP step finished
                steps_ &= ~0x80; // unmark ZLP step
                return;
            }
            
            // transfer finished
            setSuccess();
        } else {
            // transfer finished
            setSuccess(urb.actual_length);
        }
    } else {
        // error
        // canceled: ENOENT
        auto error = -status;
        setSystemError(error);
    }

    // transfer finished
    setReady();
}


// Endpoint

UsbHostDevice_io_uring::Endpoint::Endpoint(UsbHostDevice_io_uring &device, int inAddress, int outAddress)
    : BufferDevice(State::OPENING)
    , device_(device), inAddress_(inAddress), outAddress_(outAddress)
{
    device.endpoints_.add(*this);
}

UsbHostDevice_io_uring::Endpoint::~Endpoint() {
}

int UsbHostDevice_io_uring::Endpoint::getBufferCount() {
    return buffers_.count();
}

UsbHostDevice_io_uring::Buffer &UsbHostDevice_io_uring::Endpoint::getBuffer(int index) {
    return buffers_.get(index);
}

} // namespace coco

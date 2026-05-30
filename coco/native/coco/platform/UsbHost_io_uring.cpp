#include "UsbHost_io_uring.hpp"
#include <coco/convert.hpp>
#include <coco/platform/NativeFile.hpp>
#include <iostream>
#include <cstring>
#include <sys/ioctl.h>
//#include <linux/usb/ch9.h> // struct usb_device_descriptor


namespace coco {

UsbHost_io_uring::UsbHost_io_uring(Loop_io_uring &loop)
    : loop_(loop)
{
    udev_ = udev_new();
    
    mon_ = udev_monitor_new_from_netlink(udev_, "udev");

    udev_monitor_filter_add_match_subsystem_devtype(mon_, "usb", "usb_device");
    udev_monitor_enable_receiving(mon_);



    // poll for events
    int fd = udev_monitor_get_fd(mon_);
    //fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) | O_NONBLOCK);
    loop.poll(*this, fd, POLLIN);
}

UsbHost_io_uring::~UsbHost_io_uring() {
    udev_monitor_unref(mon_);
    udev_unref(udev_);
}

void UsbHost_io_uring::onCompletion(io_uring_cqe &cqe, int index) {
    if (cqe.res & POLLIN) {
        // poll again
        int fd = udev_monitor_get_fd(mon_);
        loop_.poll(*this, fd, POLLIN);

        struct udev_device* dev = udev_monitor_receive_device(mon_);
        if (dev) {
            const char *action = udev_device_get_action(dev);
            const char *syspath = udev_device_get_syspath(dev);
/*            
            const char *devtype = udev_device_get_devtype(dev);
            const char* devnode = udev_device_get_devnode(dev);
            const char* sysname = udev_device_get_sysname(dev);
            const char* vendor = udev_device_get_sysattr_value(dev, "idVendor");
            const char* product = udev_device_get_sysattr_value(dev, "idProduct");

            std::cout << "USB Event: " << (action ? action : "?") 
                        << " | Device: " << (devnode ? devnode : sysname ? sysname : "?")
                        << " | Type: " << (devtype ? devtype : "?")
                        << " | Vendor: " << (vendor ? vendor : "?")
                        << " | Product: " << (product ? product : "?") << std::endl;
*/
            if (action && syspath && std::string_view(action) == "add") {
                // get path to descriptors in sysfs
                std::string path = syspath;

                // check if device is not used yet
                if (!deviceMap_.contains(path)) {
                    // read descriptor
                    fs::path descriptorsPath = fs::path(path) / "descriptors";
                    NativeFile file(descriptorsPath, NativeFile::Mode::OPEN_READ);

                    usb::DeviceDescriptor deviceDescriptor;
                    if (file.read(0, &deviceDescriptor, sizeof(deviceDescriptor)) == sizeof(deviceDescriptor)) {
                        // check if a filter of a device accepts the device descriptor
                        for (Device &device : devices_) {
                            if (device.state_ == Device::State::OPENING && device.filter_(deviceDescriptor)) {
                                // get device path
                                const char* devnode = udev_device_get_devnode(dev);
                                if (devnode) {
                                    int handle = ::open(devnode, O_RDWR);
                                    if (handle != -1) {                   
                                        // connect and transfer ownership of handle to device
                                        device.connect(path, handle);
                                    }
                                }
                            }
                        }
                    }
                }
            }

            udev_device_unref(dev);
        }
    }
}


// Device

UsbHost_io_uring::Device::Device(UsbHost_io_uring &host, std::function<bool (const usb::DeviceDescriptor &)> filter)
    : UsbHostDevice(State::DISABLED), host_(host), filter_(filter)
{
    host.devices_.add(*this);

    
}

UsbHost_io_uring::Device::~Device() {

}

void UsbHost_io_uring::Device::open() {
    if (state_ == State::DISABLED) {
        state_ = State::OPENING;
        notify(Events::ENTER_OPENING);

        // enumerate existing devices
        struct udev_enumerate* enumerate = udev_enumerate_new(host_.udev_);
        udev_enumerate_add_match_subsystem(enumerate, "usb");
        udev_enumerate_scan_devices(enumerate);
        struct udev_list_entry* devices = udev_enumerate_get_list_entry(enumerate);
        struct udev_list_entry* entry;
        udev_list_entry_foreach(entry, devices) {
            const char* path = udev_list_entry_get_name(entry);
            struct udev_device* dev = udev_device_new_from_syspath(host_.udev_, path);
            if (dev) {                
                const char *devtype = udev_device_get_devtype(dev);
                const char *syspath = udev_device_get_syspath(dev);
                /*const char* devnode = udev_device_get_devnode(dev);
                const char* sysname = udev_device_get_sysname(dev);
                const char* vendor = udev_device_get_sysattr_value(dev, "idVendor");
                const char* product = udev_device_get_sysattr_value(dev, "idProduct");

                std::cout << "USB Device: " << (devnode ? devnode : sysname ? sysname : "?")
                            << " | Type: " << (devtype ? devtype : "?")
                            << " | Vendor: " << (vendor ? vendor : "?")
                            << " | Product: " << (product ? product : "?") << std::endl;
*/
                // read device descriptor
                if (devtype && syspath && std::string_view(devtype) == "usb_device") {
                    // get path to descriptors in sysfs
                    std::string path = syspath;
                    
                    // check if device is not used yet
                    if (!host_.deviceMap_.contains(path)) {
                        // read descriptor
                        fs::path descriptorsPath = fs::path(path) / "descriptors";
                        NativeFile file(descriptorsPath, NativeFile::Mode::OPEN_READ);

                        usb::DeviceDescriptor deviceDescriptor;
                        if (file.read(0, &deviceDescriptor, sizeof(deviceDescriptor)) == sizeof(deviceDescriptor)) {
                            if (filter_(deviceDescriptor)) {
                                // get device path
                                const char* devnode = udev_device_get_devnode(dev);
                                if (devnode) {
                                    int handle = ::open(devnode, O_RDWR);
                                    if (handle != -1) {                   
                                        // connect and transfer ownership of handle to device
                                        connect(path, handle);
                                    }
                                }
                            }
                        }
                    }
                }

                udev_device_unref(dev);
            }
        }
        udev_enumerate_unref(enumerate);    
    }
}

void UsbHost_io_uring::Device::connect(const std::string& path, int handle) {
    it_ = host_.deviceMap_.insert(std::map<std::string, Device *>::value_type{path, this}).first;
    handle_ = handle;

    int interface = 0;
    ioctl(handle, USBDEVFS_CLAIMINTERFACE, &interface);

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

    // poll read events for urb completions and disconnect
    host_.loop_.poll(*this, handle_, POLLIN | POLLOUT);
}

void UsbHost_io_uring::Device::disconnect() {
    if (state_ == Device::State::READY) {
        // erase from device map of host
        host_.deviceMap_.erase(it_);

        // close handle
        ::close(handle_);
        handle_ = -1;


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

void UsbHost_io_uring::Device::onCompletion(io_uring_cqe &cqe, int index) {
    if (cqe.res & (POLLERR | POLLHUP | POLLNVAL)) {
        // device disconnected
        disconnect();
    } else if (cqe.res & (POLLIN | POLLOUT)) {
        // poll again
        host_.loop_.poll(*this, handle_, POLLIN | POLLOUT);

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

UsbHost_io_uring::ControlBuffer::ControlBuffer(int capacity, Device &device)
    : Buffer(&setup_, sizeof(setup_), new uint8_t[capacity], capacity, device.state_)
    , device_(device)
{
    device.controlBuffers_.add(*this);
}

UsbHost_io_uring::ControlBuffer::~ControlBuffer() {
    delete [] data_;
}

bool UsbHost_io_uring::ControlBuffer::start() {
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

bool UsbHost_io_uring::ControlBuffer::cancel() {
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

bool UsbHost_io_uring::ControlBuffer::transfer() {
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

UsbHost_io_uring::Buffer::Buffer(int capacity, Endpoint &endpoint)
    : coco::Buffer(new uint8_t[capacity], capacity, endpoint.device_.state_)
    , endpoint_(endpoint)
{
    endpoint.buffers_.add(*this);

    int urbSize = sizeof(struct usbdevfs_urb) * 2;
    urb_ = static_cast<struct usbdevfs_urb*>(std::malloc(urbSize));
    urb0_ = urb_ + 1;
    memset(urb_, 0, urbSize);
}

UsbHost_io_uring::Buffer::~Buffer() {
    delete [] data_;
    std::free(urb_);
}

bool UsbHost_io_uring::Buffer::start() {
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

bool UsbHost_io_uring::Buffer::cancel() {
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

bool UsbHost_io_uring::Buffer::transfer() {
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

void UsbHost_io_uring::Buffer::onCompletion() {
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

UsbHost_io_uring::Endpoint::Endpoint(UsbHost_io_uring::Device &device, int inAddress, int outAddress)
    : BufferDevice(State::OPENING)
    , device_(device), inAddress_(inAddress), outAddress_(outAddress)
{
    device.endpoints_.add(*this);
}

UsbHost_io_uring::Endpoint::~Endpoint() {
}

int UsbHost_io_uring::Endpoint::getBufferCount() {
    return buffers_.count();
}

UsbHost_io_uring::Buffer &UsbHost_io_uring::Endpoint::getBuffer(int index) {
    return buffers_.get(index);
}

} // namespace coco

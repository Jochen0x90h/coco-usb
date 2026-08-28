#include "UsbMonitor_udev.hpp"
#include <coco/platform/NativeFile.hpp>


namespace coco {

namespace {

    const char *subsystem = "usb";
    const char *devtype = "usb_device";

    auto getAttribute(struct udev_device *dev, const char* name) {
        const char *attr = udev_device_get_sysattr_value(dev, name);
        return attr == nullptr ? String() : String(attr);
    }

} // namespace

UsbMonitor_udev::UsbMonitor_udev(Loop_io_uring &loop)
    : loop_(loop)
{
    udev_ = udev_new();
    mon_ = udev_monitor_new_from_netlink(udev_, "udev");

    udev_monitor_filter_add_match_subsystem_devtype(mon_, subsystem, devtype);
    udev_monitor_enable_receiving(mon_);

    // poll for events
    int fd = udev_monitor_get_fd(mon_);
    //fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) | O_NONBLOCK);
    loop.poll(*this, fd, POLLIN);
}

UsbMonitor_udev::~UsbMonitor_udev() {
    udev_unref(udev_);
}

void UsbMonitor_udev::listenAdd(std::function<void (const std::filesystem::path &, const usb::DeviceDescriptor &, String, String, String)> function, Action action) {
    if ((action & Action::ENUMERATE) != 0) {
        // enumerate devices
        auto udev = udev_;
        auto enumerate = udev_enumerate_new(udev);
        udev_enumerate_add_match_subsystem(enumerate, subsystem);
        udev_enumerate_scan_devices(enumerate);
        struct udev_list_entry* devices = udev_enumerate_get_list_entry(enumerate);
        struct udev_list_entry* entry;
        udev_list_entry_foreach(entry, devices) {
            const char* path = udev_list_entry_get_name(entry);
            struct udev_device* dev = udev_device_new_from_syspath(udev, path);
            if (dev) {
                const char *devtype = udev_device_get_devtype(dev);
                const char *syspath = udev_device_get_syspath(dev);
                const char* devnode = udev_device_get_devnode(dev);

                // read device descriptor
                if (devtype && syspath && devnode && String(devtype) == devtype) {
                    fs::path descriptorsPath = fs::path(syspath) / "descriptors";

                    // read descriptor
                    NativeFile file(descriptorsPath, NativeFile::Mode::OPEN_READ);

                    usb::DeviceDescriptor deviceDescriptor;
                    if (file.read(0, &deviceDescriptor, sizeof(deviceDescriptor)) == sizeof(deviceDescriptor)) {
                        // call user function
                        function(devnode, deviceDescriptor,
                            getAttribute(dev, "manufacturer"),
                            getAttribute(dev, "product"),
                            getAttribute(dev, "serial"));
                    }
                }
                udev_device_unref(dev);
            }
        }
        udev_enumerate_unref(enumerate);
    }
    if ((action & Action::MONITOR) != 0) {
        addListeners_.push_back(function);
    }
}

void UsbMonitor_udev::listenRemove(std::function<void (const std::filesystem::path &)> function) {
    removeListeners_.push_back(function);
}


void UsbMonitor_udev::onCompletion(io_uring_cqe &cqe, int index) {
    if (cqe.res & POLLIN) {
        // poll again
        int fd = udev_monitor_get_fd(mon_);
        loop_.poll(*this, fd, POLLIN);

        struct udev_device* dev = udev_monitor_receive_device(mon_);
        if (dev) {
            const char *action = udev_device_get_action(dev);
            const char *syspath = udev_device_get_syspath(dev);
            const char* devnode = udev_device_get_devnode(dev);
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
            if (action && syspath && devnode) {
                if (String(action) == "add") {
                    // read descriptor
                    fs::path descriptorsPath = fs::path(syspath) / "descriptors";
                    NativeFile file(descriptorsPath, NativeFile::Mode::OPEN_READ);
                    usb::DeviceDescriptor deviceDescriptor;
                    if (file.read(0, &deviceDescriptor, sizeof(deviceDescriptor)) == sizeof(deviceDescriptor)) {
                        // call add listeners
                        for (auto &function : addListeners_) {
                            function(devnode, deviceDescriptor,
                                getAttribute(dev, "manufacturer"),
                                getAttribute(dev, "product"),
                                getAttribute(dev, "serial"));
                        }
                    }
                } else if (String(action) == "remove") {
                    // call remove listeners
                    for (auto &function : removeListeners_) {
                        function(devnode);
                    }
                }
            }

            udev_device_unref(dev);
        }
    }
}

} // namespace coco

#pragma once

#include <coco/String.hpp>
#include <coco/usb.hpp>
#include <coco/UsbMonitor.hpp>
#include <coco/platform/Loop_io_uring.hpp>
#include <libudev.h>


namespace coco {

/// @brief Implementation of UartMonitor using Linux udev.
///
class UsbMonitor_udev : public UsbMonitor, public Loop_io_uring::CompletionHandler {
public:

    UsbMonitor_udev(Loop_io_uring &loop);

    ~UsbMonitor_udev() override;

    // UsbMonitor methods
    void listenAdd(std::function<void (DevicePath, const usb::DeviceDescriptor &, String, String, String)> function, Action action = Action::ENUMERATE_MONITOR) override;
    void listenRemove(std::function<void (DevicePath)>) override;

protected:
    void onCompletion(io_uring_cqe &cqe, int index) override;

    Loop_io_uring &loop_;

    struct udev *udev_;
    struct udev_monitor* mon_;

    std::vector<std::function<void (DevicePath, const usb::DeviceDescriptor &, String, String, String)>> addListeners_;
    std::vector<std::function<void (DevicePath)>> removeListeners_;
};

} // namespace coco

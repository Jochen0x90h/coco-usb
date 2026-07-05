#pragma once

#include "usb.hpp"
#include <coco/enum.hpp>
#include <filesystem>
#include <functional>


namespace coco {

/// @brief Monitor for USB devices.
/// When a device is added or removed, a callback gets invoked.
class UsbMonitor {
public:
    enum class Action {
        ENUMERATE = 1,
        MONITOR = 2,
        ENUMERATE_MONITOR = 3
    };

    virtual ~UsbMonitor() {};

    /// @brief Listen on add events
    /// @param action Action to perform (enumerate, monitor or both)
    /// @param function Callback function with path to device, descriptor, manufacturer, product and serial number
    virtual void listenAdd(std::function<void (const std::filesystem::path &, const usb::DeviceDescriptor &, String, String, String)> function, Action action =
        Action::ENUMERATE_MONITOR) = 0;

    /// @brief Listen on remove events
    /// @param action Action to perform (enumerate, monitor or both)
    /// @param function Callback function with path to device
    virtual void listenRemove(std::function<void (const std::filesystem::path &)>) = 0;
};
COCO_ENUM(UsbMonitor::Action);

} // namespace coco

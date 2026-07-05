#pragma once

#include <coco/String.hpp>
#include <coco/usb.hpp>
#include <coco/UsbMonitor.hpp>
#include <coco/platform/Loop_Win32.hpp> // includes Windows.h
#include <map>


namespace coco {

/// @brief Implementation of UsbMonitor using SetupAPI and WinUSB.
/// Polls every second for new devices.
class UsbMonitor_SetupAPI : public UsbMonitor, public Loop_Win32::TimeoutHandler {
public:

    UsbMonitor_SetupAPI(Loop_Win32 &loop);

    ~UsbMonitor_SetupAPI() override;

    void listenAdd(std::function<void (const std::filesystem::path &, const usb::DeviceDescriptor &, String, String, String)> function, Action action = Action::ENUMERATE_MONITOR) override;
    void listenRemove(std::function<void (const std::filesystem::path &)>);

protected:
    void onTimeout() override;

    Loop_Win32 &loop_;

    struct DeviceInfo {
        usb::DeviceDescriptor descriptor;
        std::string manufacturer;
        std::string product;
        std::string serialNumber;

        // flag for "garbage collection" of devices
        bool flag;
    };
    std::map<std::filesystem::path, DeviceInfo> deviceInfos_;

    std::vector<std::function<void (const std::filesystem::path &, const usb::DeviceDescriptor &, String, String, String)>> addListeners_;
    std::vector<std::function<void (const std::filesystem::path &)>> removeListeners_;
};

} // namespace coco

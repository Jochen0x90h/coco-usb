#pragma once

#include <coco/String.hpp>
#include <coco/usb.hpp>
#include <coco/UsbMonitor.hpp>
#include <coco/platform/Loop_Win32.hpp> // includes Windows.h
#include <map>


namespace coco {

/// @brief Implementation of UsbMonitor using SetupAPI, WinUSB and a message window.
/// Note that currently only one instance can exist because of the window class for the message window.
class UsbMonitor_SetupAPI : public UsbMonitor, public Loop_Win32::DeviceHandler {
public:

    UsbMonitor_SetupAPI(Loop_Win32 &loop);

    ~UsbMonitor_SetupAPI() override;

    // UsbMonitor methods
    void listenAdd(std::function<void (DevicePath, const usb::DeviceDescriptor &, String, String, String)> function, Action action = Action::ENUMERATE_MONITOR) override;
    void listenRemove(std::function<void (DevicePath)>) override;

protected:
    //static LRESULT CALLBACK DeviceWndProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam);
    // DeviceHandler methods
    void onDeviceChange(Loop_Win32::DeviceType type, bool add, DevicePath path) override;

    Loop_Win32 &loop_;

    HWND window_;

    std::vector<std::function<void (DevicePath, const usb::DeviceDescriptor &, String, String, String)>> addListeners_;
    std::vector<std::function<void (DevicePath)>> removeListeners_;
};

} // namespace coco

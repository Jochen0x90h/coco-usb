#include <coco/platform/WindowsDef.hpp>
#include <windows.h>
#include <winusb.h>
#include <setupapi.h>
#include <initguid.h>
#include <usbiodef.h>
#include <coco/platform/WindowsUndef.hpp>

#include "UsbMonitor_SetupAPI.hpp"
#include <coco/debug.hpp>


namespace coco {

namespace {

    // buffer for device path and string descriptor
    union Buffer {
        SP_DEVICE_INTERFACE_DETAIL_DATA_W devicePath;
        usb::StringDescriptor<128> stringDescriptor;

        Buffer() {};
    };

    std::string readStringDescriptor(WINUSB_INTERFACE_HANDLE interface, int id, usb::StringDescriptor<128> &descriptor) {
        ULONG transferred;
        bool result = WinUsb_GetDescriptor(interface, int(usb::DescriptorType::STRING), id, 0x0409,
            (UCHAR *)&descriptor, sizeof(descriptor), &transferred);
        if (transferred <= 2)
            return {};

        char16_t *utf16Str = descriptor.wString;
        int charCount = (descriptor.bLength - 2) / 2;
        while (charCount > 0 && utf16Str[charCount - 1] == 0 || utf16Str[charCount - 1] == ' ')
            --charCount;
        if (charCount <= 0)
            return {};

        int utf8Length = WideCharToMultiByte(CP_UTF8, 0, (wchar_t *)utf16Str, charCount, nullptr, 0, nullptr, nullptr);
        if (utf8Length <= 0)
            return {};

        // convert to std::string
        std::string utf8Result(utf8Length, '\0');
        WideCharToMultiByte(CP_UTF8, 0, (wchar_t *)utf16Str, charCount, utf8Result.data(), utf8Length, nullptr, nullptr);
        return utf8Result;
    }

} // namespace

UsbMonitor_SetupAPI::UsbMonitor_SetupAPI(Loop_Win32 &loop)
    : loop_(loop)
{
    onTimeout();
}

UsbMonitor_SetupAPI::~UsbMonitor_SetupAPI() {
}

void UsbMonitor_SetupAPI::listenAdd(std::function<void (const std::filesystem::path &, const usb::DeviceDescriptor &, String, String, String)> function, Action action) {
    if ((action & Action::ENUMERATE) != 0) {
        for (auto &p : deviceInfos_) {
            auto &deviceInfo = p.second;
            auto &descriptor = deviceInfo.descriptor;
            if (descriptor.idProduct != 0 && descriptor.idVendor != 0) {
                // call user function
                function(p.first, descriptor, deviceInfo.manufacturer, deviceInfo.product, deviceInfo.serialNumber);
            }
        }
    }
    if ((action & Action::MONITOR) != 0) {
        addListeners_.push_back(function);
    }
}

void UsbMonitor_SetupAPI::listenRemove(std::function<void (const std::filesystem::path &)> function) {
    removeListeners_.push_back(function);
}

void UsbMonitor_SetupAPI::onTimeout() {
    // restart timeout
    loop_.invoke(*this, 1s);

    // flag all devices
    for (auto &p : deviceInfos_) {
        p.second.flag = true;
    }

    // enumerate devices
    HDEVINFO devs = SetupDiGetClassDevsW(nullptr, nullptr, nullptr, DIGCF_ALLCLASSES | DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);
    int index = 0;
    SP_DEVINFO_DATA deviceData;
    deviceData.cbSize = sizeof(SP_DEVINFO_DATA);
    while (SetupDiEnumDeviceInfo(devs, index, &deviceData)) {
        ++index;

        // get interface data
        SP_DEVICE_INTERFACE_DATA interfaceData;
        interfaceData.cbSize = sizeof(SP_DEVICE_INTERFACE_DATA);
        if (!SetupDiEnumDeviceInterfaces(devs, &deviceData, &GUID_DEVINTERFACE_USB_DEVICE, 0, &interfaceData)) {
            continue;
        }

        // buffer for device path and string descriptor
        Buffer buffer;

        // get device path
        buffer.devicePath.cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA_W);
        SP_DEVINFO_DATA devInfoData;
        devInfoData.cbSize = sizeof(SP_DEVINFO_DATA);
        if (!SetupDiGetDeviceInterfaceDetailW(devs, &interfaceData, &buffer.devicePath, sizeof(buffer), nullptr, &devInfoData)) {
            // error
            continue;
        }

        // check if device is new
        wchar_t *path = buffer.devicePath.DevicePath;
        auto [it, inserted] = deviceInfos_.emplace(path, DeviceInfo{});
        auto &deviceInfo = it->second;
        debug::out << it->first.string() << '\n';

        if (inserted) {
            // found a new device, path has the form \\?\usb#vid_1915&pid_1337#5&41045ef&0&4#{a5dcbf10-6530-11d2-901f-00c04fb951ed}
            auto &descriptor = deviceInfo.descriptor;

            // try to open the device
            auto handle = CreateFileW(it->first.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, FILE_FLAG_OVERLAPPED, nullptr);
            if (handle == INVALID_HANDLE_VALUE)
                continue;

            // get USB interface
            WINUSB_INTERFACE_HANDLE interface;
            if (!WinUsb_Initialize(handle, &interface)) {
                CloseHandle(handle);
                continue;
            }

            // read device descriptor
            ULONG transferred;
            bool result = WinUsb_GetDescriptor(interface, int(usb::DescriptorType::DEVICE), 0, 0,
                (UCHAR*)&descriptor, sizeof(descriptor), &transferred);
            if (!result || transferred < sizeof(descriptor)) {
                WinUsb_Free(interface);
                CloseHandle(handle);
                continue;
            }

            // read string descriptors
            deviceInfo.manufacturer = readStringDescriptor(interface, descriptor.iManufacturer, buffer.stringDescriptor);
            deviceInfo.product = readStringDescriptor(interface, descriptor.iProduct, buffer.stringDescriptor);
            deviceInfo.serialNumber = readStringDescriptor(interface, descriptor.iSerialNumber, buffer.stringDescriptor);

            // call add listeners
            for (auto &function : addListeners_) {
                function(it->first, descriptor,
                    deviceInfo.manufacturer,
                    deviceInfo.product,
                    deviceInfo.serialNumber);
            }
        }
        deviceInfo.flag = false;
    }

    // detect removed devices
    auto it = deviceInfos_.begin();
    while (it != deviceInfos_.end()) {
        auto current = it;
        ++it;
        if (current->second.flag) {
            auto &deviceInfo = current->second;
            if (deviceInfo.descriptor.idProduct != 0 && deviceInfo.descriptor.idVendor != 0) {
                // call remove listeners
                for (auto &function : removeListeners_) {
                    function(current->first);
                }
            }

            // erase
            deviceInfos_.erase(current);
        }
    }
}

} // namespace coco

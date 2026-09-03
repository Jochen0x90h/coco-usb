#include <coco/platform/WindowsDef.hpp>
#include <windows.h>
#include <winusb.h>
#include <SetupAPI.h>
#include <initguid.h>
#include <usbiodef.h>
#include <Dbt.h>
#include <coco/platform/WindowsUndef.hpp>

#include "UsbMonitor_SetupAPI.hpp"
#include <coco/debug.hpp>


namespace coco {

namespace {

    // buffer for device path and string descriptors
    union Buffer {
        SP_DEVICE_INTERFACE_DETAIL_DATA_W devicePath;
        wchar_t stringBuffer[128];
        Buffer() {};
    };

    std::string readStringDescriptor(WINUSB_INTERFACE_HANDLE interface, int id) {
        usb::StringDescriptor<128> descriptor;
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
    /*
    // register window class
    WNDCLASSEXW wc = {};
    wc.cbSize = sizeof(wc);
    wc.lpfnWndProc = &UsbMonitor_SetupAPI::DeviceWndProc;
    wc.hInstance = GetModuleHandle(nullptr);
    wc.lpszClassName = L"UsbMonitor";
    auto result = RegisterClassExW(&wc);

    // create message only window
    window_ = CreateWindowExW(
        0,                              // no extended style
        L"UsbMonitor",
        L"",                            // no title
        0,                              // no styles (no WS_VISIBLE!)
        0, 0, 0, 0,                     // position/size
        HWND_MESSAGE,                   // message only window
        nullptr,
        GetModuleHandle(nullptr),
        this
    );
    SetWindowLongPtrW(window_, GWLP_USERDATA, (LONG_PTR)this);

    // register device notifications
    DEV_BROADCAST_DEVICEINTERFACE_W filter = {};
    filter.dbcc_size = sizeof(filter);
    filter.dbcc_devicetype = DBT_DEVTYP_DEVICEINTERFACE;
    filter.dbcc_classguid  = GUID_DEVINTERFACE_USB_DEVICE;
    RegisterDeviceNotificationW(
        window_,
        &filter,
        DEVICE_NOTIFY_WINDOW_HANDLE
    );
    */
   loop.addDeviceHandler(*this);
}

UsbMonitor_SetupAPI::~UsbMonitor_SetupAPI() {
    DestroyWindow(window_);
}

void UsbMonitor_SetupAPI::listenAdd(std::function<void (DevicePath, const usb::DeviceDescriptor &, String, String, String)> function, Action action) {
    if ((action & Action::ENUMERATE) != 0) {
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

            // buffer for device path and string descriptors
            Buffer buffer;

            // get device path
            buffer.devicePath.cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA_W);
            SP_DEVINFO_DATA devInfoData;
            devInfoData.cbSize = sizeof(SP_DEVINFO_DATA);
            if (!SetupDiGetDeviceInterfaceDetailW(devs, &interfaceData, &buffer.devicePath, sizeof(buffer), nullptr, &devInfoData)) {
                // error
                continue;
            }

            // found a new device, path has the form \\?\usb#vid_1915&pid_1337#5&41045ef&0&4#{a5dcbf10-6530-11d2-901f-00c04fb951ed}
            DevicePath path = (wchar_t *)buffer.devicePath.DevicePath;
            usb::DeviceDescriptor descriptor;

            // try to open the device
            auto handle = CreateFileW(path.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, FILE_FLAG_OVERLAPPED, nullptr);
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
            std::string manufacturer = readStringDescriptor(interface, descriptor.iManufacturer);
            std::string product = readStringDescriptor(interface, descriptor.iProduct);
            std::string serialNumber = readStringDescriptor(interface, descriptor.iSerialNumber);

            // close device
            WinUsb_Free(interface);
            CloseHandle(handle);

            // call user function
            function(path, descriptor, manufacturer, product, serialNumber);
        }
    }
    if ((action & Action::MONITOR) != 0) {
        addListeners_.push_back(function);
    }
}

void UsbMonitor_SetupAPI::listenRemove(std::function<void (DevicePath)> function) {
    removeListeners_.push_back(function);
}

void UsbMonitor_SetupAPI::onDeviceChange(Loop_Win32::DeviceType type, bool add, DevicePath path) {
    if (type != Loop_Win32::DeviceType::USB)
        return;

    if (add) {
        // try to open the device
        auto handle = CreateFileW(path.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, FILE_FLAG_OVERLAPPED, nullptr);
        if (handle == INVALID_HANDLE_VALUE)
            return;

        // get USB interface
        WINUSB_INTERFACE_HANDLE interface;
        if (!WinUsb_Initialize(handle, &interface)) {
            CloseHandle(handle);
            return;
        }

        // read device descriptor
        usb::DeviceDescriptor descriptor;
        ULONG transferred;
        bool result = WinUsb_GetDescriptor(interface, int(usb::DescriptorType::DEVICE), 0, 0,
            (UCHAR*)&descriptor, sizeof(descriptor), &transferred);
        if (!result || transferred < sizeof(descriptor)) {
            WinUsb_Free(interface);
            CloseHandle(handle);
            return;
        }

        // buffer for string descriptors
        Buffer buffer;

        // read string descriptors
        std::string manufacturer = readStringDescriptor(interface, descriptor.iManufacturer);
        std::string product = readStringDescriptor(interface, descriptor.iProduct);
        std::string serialNumber = readStringDescriptor(interface, descriptor.iSerialNumber);

        // close device
        WinUsb_Free(interface);
        CloseHandle(handle);

        // call add listeners
        for (auto &function : addListeners_) {
            function(path,
                descriptor,
                manufacturer,
                product,
                serialNumber);
        }
    } else {
        // call remove listeners
        for (auto &function : removeListeners_) {
            function(path);
        }
    }
}
/*
LRESULT CALLBACK UsbMonitor_SetupAPI::DeviceWndProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam) {
    switch (msg) {
    case WM_DEVICECHANGE:
        {
            UsbMonitor_SetupAPI *monitor = (UsbMonitor_SetupAPI *)GetWindowLongPtrW(hwnd, GWLP_USERDATA);
            auto *iface = (PDEV_BROADCAST_DEVICEINTERFACE_W)lParam;
            if (iface && iface->dbcc_devicetype == DBT_DEVTYP_DEVICEINTERFACE) {
                std::filesystem::path path = (wchar_t *)iface->dbcc_name;
                if (wParam == DBT_DEVICEARRIVAL) {
                    usb::DeviceDescriptor descriptor;

                    // try to open the device
                    auto handle = CreateFileW(path.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, FILE_FLAG_OVERLAPPED, nullptr);
                    if (handle == INVALID_HANDLE_VALUE)
                        return TRUE;

                    // get USB interface
                    WINUSB_INTERFACE_HANDLE interface;
                    if (!WinUsb_Initialize(handle, &interface)) {
                        CloseHandle(handle);
                        return TRUE;
                    }

                    // read device descriptor
                    ULONG transferred;
                    bool result = WinUsb_GetDescriptor(interface, int(usb::DescriptorType::DEVICE), 0, 0,
                        (UCHAR*)&descriptor, sizeof(descriptor), &transferred);
                    if (!result || transferred < sizeof(descriptor)) {
                        WinUsb_Free(interface);
                        CloseHandle(handle);
                        return TRUE;
                    }

                    // buffer for string descriptors
                    Buffer buffer;

                    // read string descriptors
                    std::string manufacturer = readStringDescriptor(interface, descriptor.iManufacturer, buffer.stringDescriptor);
                    std::string product = readStringDescriptor(interface, descriptor.iProduct, buffer.stringDescriptor);
                    std::string serialNumber = readStringDescriptor(interface, descriptor.iSerialNumber, buffer.stringDescriptor);

                    // close device
                    WinUsb_Free(interface);
                    CloseHandle(handle);

                    // call add listeners
                    for (auto &function : monitor->addListeners_) {
                        function(path, descriptor,
                            manufacturer,
                            product,
                            serialNumber);
                    }

                } else if (wParam == DBT_DEVICEREMOVECOMPLETE) {
                    // call remove listeners
                    for (auto &function : monitor->removeListeners_) {
                        function(path);
                    }
                }
            }
        }
        return TRUE;
    case WM_DESTROY:
        PostQuitMessage(0);
        return 0;
    }
    return DefWindowProc(hwnd, msg, wParam, lParam);
}*/

} // namespace coco

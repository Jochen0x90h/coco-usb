#pragma once

#include <coco/BufferDevice.hpp>
#include <coco/UsbHostDevice.hpp>
#include <coco/usb.hpp>
#include <coco/platform/Loop_Win32.hpp> // includes Windows.h
#include <winusb.h>
#include <string>
#include <map>
#include <functional>
#include <filesystem>


namespace coco {

/// @brief USB host implementation using Win32 and WinUsb
/// https://learn.microsoft.com/en-us/windows/win32/api/winusb/
/// Note that currently only one instance can exist because of the window class for the message window.
class UsbHost_WinUSB {
public:
    UsbHost_WinUSB(Loop_Win32 &loop);
    ~UsbHost_WinUSB();


    class Device;

    /// @brief Buffer for control transfers
    /// Max size is 512 according to spec: https://www.techdesignforums.com/practice/technique/usb-3-0-protocol-layer-2/
    class ControlBuffer : public coco::Buffer, public coco::IntrusiveListNode {
        friend class Device;
    public:
        ControlBuffer(int capacity, Device &device);
        ~ControlBuffer() override;

        // Buffer methods
        bool start() override;
        bool cancel() override;

    protected:
        bool transfer();
        void onCompletion(OVERLAPPED *overlapped);

        Device &device_;
        WINUSB_SETUP_PACKET setup_;
        OVERLAPPED overlapped_;
    };


    class Endpoint;

    /// @brief Buffer for transferring data to/from an endpoint
    ///
    class Buffer : public coco::Buffer, public coco::IntrusiveListNode {
        friend class UsbHost_WinUSB::Device;
    public:
        Buffer(int capacity, Endpoint &endpoint);
        ~Buffer() override;

        // Buffer methods
        bool start() override;
        bool cancel() override;

    protected:
        bool transfer();
        void onCompletion(OVERLAPPED *overlapped);

        Endpoint &endpoint_;

        OVERLAPPED overlapped_[2];
        int index_;
    };

    /// @brief Bulk/Interrupt endpoint
    ///
    class Endpoint : public BufferDevice, public coco::IntrusiveListNode {
        friend class UsbHost_WinUSB::Device;
        friend class Buffer;
    public:
        Endpoint(UsbHost_WinUSB::Device &device, int inAddress, int outAddress);
        Endpoint(UsbHost_WinUSB::Device &device, int address) : Endpoint(device,  usb::IN | address, usb::OUT | address) {}
        ~Endpoint();

        // BufferDevice methods
        int getBufferCount() override;
        Buffer &getBuffer(int index) override;

    protected:
        UsbHost_WinUSB::Device &device_;
        int inAddress_;
        int outAddress_;

        // list of buffers
        IntrusiveList<Buffer> buffers_;
    };

    /// @brief USB device as seen by the host.
    /// Connects itself to an actual USB device when it is plugged in and the filter on the device descriptor returns
    /// true.
    class Device : public UsbHostDevice, public Loop_Win32::CompletionHandler, public coco::IntrusiveListNode {
        friend class UsbHost_WinUSB;
    public:
        /// @brief Constructor.
        /// @param host usb host
        /// @param filter Filter to indicate if an usb device is handled by this device instance
        Device(UsbHost_WinUSB &host);

        ~Device() override;

        /// @brief Open device by path.
        /// Fails if already open. Calling close() does nothing if the device is not open.
        bool open(const std::filesystem::path &path);

        void close() override;

        //void getDescriptor(usb::DescriptorType type, void *data, int &size) override;

        //void open() override;

    protected:
        //void connect(HANDLE handle, void *interface);
        //void disconnect();
        void onCompletion(OVERLAPPED *overlapped) override;

        UsbHost_WinUSB &host_;

        // iterator of device list and flag for remove detection
        std::map<std::string, Device *>::iterator it_;
        bool flag_;

        // device path, needed for removal
        std::filesystem::path path_;

        // device handle and WinUsb interface
        HANDLE handle_ = INVALID_HANDLE_VALUE;
        WINUSB_INTERFACE_HANDLE interface_ = nullptr;

        // list of all buffers for control endpoint
        IntrusiveList<ControlBuffer> controlBuffers_;

        // list of all bulk/interrupt endpoints
        IntrusiveList<Endpoint> endpoints_;
    };

protected:
    //void onTimeout() override;
    static LRESULT CALLBACK DeviceWndProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam);

    Loop_Win32 &loop_;

    // device monitor for disconnecting devices
    HWND window_;

    IntrusiveList<Device> devices_;
};

} // namespace coco

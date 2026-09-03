#pragma once

#include <coco/BufferDevice.hpp>
#include <coco/usb.hpp>
#include <coco/platform/Loop_Win32.hpp> // includes Windows.h
#include <winusb.h>
#include <string>
#include <map>
#include <functional>
#include <filesystem>


namespace coco {

/// @brief USB device as seen by the host.
/// Use UsbMonitor to wait for the connection of your device, then use open() to connect to the device.
/// The device will disconnect automatically when it is removed.
class UsbHostDevice_WinUSB : public Device, public Loop_Win32::CompletionHandler, public Loop_Win32::DeviceHandler {
    friend class UsbHost_WinUSB;
public:
    /// @brief Constructor.
    /// @param loop Event loop
    UsbHostDevice_WinUSB(Loop_Win32 &loop);

    ~UsbHostDevice_WinUSB() override;

    /// @brief Open device by path.
    /// Calling open() or close() on a device that is already open or closed does nothing.
    /// @param path Path to device, e.g. obtained from UsbMonitor
    /// @return true when successful
    bool open(DevicePath path);
    void close() override;

    //void getDescriptor(usb::DescriptorType type, void *data, int &size) override;


    /// @brief Buffer for control transfers
    /// Max size is 512 according to spec: https://www.techdesignforums.com/practice/technique/usb-3-0-protocol-layer-2/
    class ControlBuffer : public coco::Buffer, public coco::IntrusiveListNode {
        friend class UsbHostDevice_WinUSB;
    public:
        ControlBuffer(int capacity, UsbHostDevice_WinUSB &device);
        ~ControlBuffer() override;

        // Buffer methods
        bool start() override;
        bool cancel() override;

    protected:
        bool transfer();
        void onCompletion(OVERLAPPED *overlapped);

        UsbHostDevice_WinUSB &device_;
        WINUSB_SETUP_PACKET setup_;
        OVERLAPPED overlapped_;
    };


    class Endpoint;

    /// @brief Buffer for transferring data to/from an endpoint
    ///
    class Buffer : public coco::Buffer, public coco::IntrusiveListNode {
        friend class UsbHostDevice_WinUSB;
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
        friend class UsbHostDevice_WinUSB;
        friend class Buffer;
    public:
        Endpoint(UsbHostDevice_WinUSB &device, int inAddress, int outAddress);
        Endpoint(UsbHostDevice_WinUSB &device, int address) : Endpoint(device,  usb::IN | address, usb::OUT | address) {}
        ~Endpoint();

        // BufferDevice methods
        int getBufferCount() override;
        Buffer &getBuffer(int index) override;

    protected:
        UsbHostDevice_WinUSB &device_;
        int inAddress_;
        int outAddress_;

        // list of buffers
        IntrusiveList<Buffer> buffers_;
    };


protected:
    // Loop_Win32::CompletionHandler methods
    void onCompletion(OVERLAPPED *overlapped) override;

    // Loop_Win32::DeviceHandler methods
    void onDeviceChange(Loop_Win32::DeviceType type, bool add, DevicePath path) override;

    Loop_Win32 &loop_;

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

} // namespace coco

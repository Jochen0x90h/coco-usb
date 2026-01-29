#pragma once

#include <coco/usb.hpp>
#include <coco/BufferDevice.hpp>
#include <coco/platform/Loop_Win32.hpp> // includes Windows.h
#include <winusb.h>
#include <string>
#include <map>
#include <functional>


namespace coco {

/// @brief USB host implementation using Win32 and WinUsb
/// https://learn.microsoft.com/en-us/windows/win32/api/winusb/
class UsbHost_WinUSB {
public:
    UsbHost_WinUSB(Loop_Win32 &loop);
    //~UsbHost_WinUSB() override;


    class Device;

    /// @brief Buffer for control transfers
    /// Max size is 512 according to spec: https://www.techdesignforums.com/practice/technique/usb-3-0-protocol-layer-2/
    class ControlBuffer : public coco::Buffer, public IntrusiveListNode, public IntrusiveListNode2 {
        friend class Device;
    public:
        ControlBuffer(int capacity, Device &device);
        ~ControlBuffer() override;

        // Buffer methods
        bool start(Op op) override;
        bool cancel() override;

    protected:
        void start();
        void handle(OVERLAPPED *overlapped);

        Device &device_;
        WINUSB_SETUP_PACKET setup_;
        OVERLAPPED overlapped_;
    };


    class Endpoint;

    /// @brief Buffer for transferring data to/from an endpoint
    ///
    class Buffer : public coco::Buffer, public IntrusiveListNode, public IntrusiveListNode2 {
        friend class UsbHost_WinUSB::Device;
    public:
        Buffer(int capacity, Endpoint &endpoint);
        ~Buffer() override;

        // Buffer methods
        bool start(Op op) override;
        bool cancel() override;

    protected:
        void start();
        void handle(OVERLAPPED *overlapped);

        Endpoint &endpoint_;

        Op op_;
        OVERLAPPED overlapped_[2];
        int index_;
    };

    /// @brief Bulk/Interrupt endpoint
    ///
    class Endpoint : public BufferDevice, public IntrusiveListNode {
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

    /// @brief USB device as seen by the host. Connects itself to an actual USB device when it is plugged in and the
    /// filter on the device descriptor returns true.
    class Device : public coco::Device, public Loop_Win32::CompletionHandler, public IntrusiveListNode {
        friend class UsbHost_WinUSB;
    public:
        /// @brief Constructor
        /// @param host usb host
        /// @param filter filter to indicate if an usb device is handled by this device instance
        Device(UsbHost_WinUSB &host, std::function<bool (const usb::DeviceDescriptor &)> filter);

        ~Device() override;

        //void getDescriptor(usb::DescriptorType type, void *data, int &size) override;

    protected:
        void connect(HANDLE file, void *interface);
        void disconnect();
        void handle(OVERLAPPED *overlapped) override;

        UsbHost_WinUSB &host_;
        std::function<bool (const usb::DeviceDescriptor &)> filter_;

        // iterator of device list and flag for remove detection
        std::map<std::string, Device *>::iterator it_;
        bool flag_;

        // file handle and WinUsb interface
        HANDLE file_ = INVALID_HANDLE_VALUE;
        WINUSB_INTERFACE_HANDLE interface_ = nullptr;

        // list of all buffers for control endpoint
        IntrusiveList<ControlBuffer> controlBuffers_;

        // list of all bulk/interrupt endpoints
        IntrusiveList<Endpoint> endpoints_;

        // pending transfers (needed to queue transfers when device is in OPENING or PAUSE state)
        IntrusiveList2<ControlBuffer> controlTransfers_;
        IntrusiveList2<Buffer> transfers_;
    };

protected:
    void handle();

    Loop_Win32 &loop_;
    TimedTask<Callback> callback_;
    std::map<std::string, Device *> deviceInfos_;
    IntrusiveList<Device> devices_;
};

} // namespace coco

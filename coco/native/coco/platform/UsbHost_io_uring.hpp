#pragma once

#include <coco/BufferDevice.hpp>
#include <coco/UsbHostDevice.hpp>
#include <coco/usb.hpp>
#include <coco/platform/Loop_io_uring.hpp>
#include <string>
#include <map>
#include <functional>
#include <libudev.h>
#include <linux/usbdevice_fs.h>


namespace coco {

/// @brief USB host implementation using io_uring and libudev
class UsbHost_io_uring : public Loop_io_uring::CompletionHandler {
public:
    UsbHost_io_uring(Loop_io_uring &loop);
    ~UsbHost_io_uring() override;


    class Device;

    /// @brief Buffer for control transfers
    /// Max size is 512 according to spec: https://www.techdesignforums.com/practice/technique/usb-3-0-protocol-layer-2/
    class ControlBuffer : public coco::Buffer, public coco::IntrusiveListNode  {
        friend class Device;
    public:
        ControlBuffer(int capacity, Device &device);
        ~ControlBuffer() override;

        // Buffer methods
        bool start() override;
        bool cancel() override;

    protected:
        bool transfer();
        //void onCompletion(io_uring_cqe &cqe, int index) override;

        usb::Setup setup_;
        Device &device_;
    };


    class Endpoint;

    /// @brief Buffer for transferring data to/from an endpoint
    ///
    class Buffer : public coco::Buffer, public coco::IntrusiveListNode {
        friend class UsbHost_io_uring::Device;
    public:
        Buffer(int capacity, Endpoint &endpoint);
        ~Buffer() override;

        // Buffer methods
        bool start() override;
        bool cancel() override;

    protected:
        bool transfer();
        void onCompletion();

        Endpoint &endpoint_;

        struct usbdevfs_urb *urb_;
        struct usbdevfs_urb *urb0_;
    };

    /// @brief Bulk/Interrupt endpoint
    ///
    class Endpoint : public BufferDevice, public coco::IntrusiveListNode {
        friend class UsbHost_io_uring::Device;
        friend class Buffer;
    public:
        Endpoint(UsbHost_io_uring::Device &device, int inAddress, int outAddress);
        Endpoint(UsbHost_io_uring::Device &device, int address) : Endpoint(device,  usb::IN | address, usb::OUT | address) {}
        ~Endpoint();

        // BufferDevice methods
        int getBufferCount() override;
        Buffer &getBuffer(int index) override;

    protected:
        UsbHost_io_uring::Device &device_;
        int inAddress_;
        int outAddress_;

        // list of buffers
        IntrusiveList<Buffer> buffers_;
    };

    /// @brief USB device as seen by the host.
    /// Connects itself to an actual USB device when it is plugged in and the filter on the device descriptor returns
    /// true.
    class Device : public UsbHostDevice, public Loop_io_uring::CompletionHandler, public coco::IntrusiveListNode {
        friend class UsbHost_io_uring;
    public:
        /// @brief Constructor.
        /// @param host usb host
        /// @param filter Filter to indicate if an usb device is handled by this device instance
        Device(UsbHost_io_uring &host, std::function<bool (const usb::DeviceDescriptor &)> filter);

        ~Device() override;

        //void getDescriptor(usb::DescriptorType type, void *data, int &size) override;

        void open() override;

    protected:
        void connect(const std::string& path, int handle);
        void disconnect();
        void onCompletion(io_uring_cqe &cqe, int index) override;

        UsbHost_io_uring &host_;
        std::function<bool (const usb::DeviceDescriptor &)> filter_;

        // iterator of device map
        std::map<std::string, Device *>::iterator it_;

        // device handle
        int handle_ = -1;

        // list of all buffers for control endpoint
        IntrusiveList<ControlBuffer> controlBuffers_;

        // list of all bulk/interrupt endpoints
        IntrusiveList<Endpoint> endpoints_;
    };

protected:
    void onCompletion(io_uring_cqe &cqe, int index) override;

    Loop_io_uring &loop_;
    struct udev* udev_;
    struct udev_monitor* mon_;
    std::map<std::string, Device *> deviceMap_; // device path -> device instance
    IntrusiveList<Device> devices_;
};

} // namespace coco

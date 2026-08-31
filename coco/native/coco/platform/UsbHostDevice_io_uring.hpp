#pragma once

#include <coco/BufferDevice.hpp>
#include <coco/usb.hpp>
#include <coco/platform/Loop_io_uring.hpp>
#include <string>
#include <map>
#include <filesystem>
#include <functional>
#include <libudev.h>
#include <linux/usbdevice_fs.h>


namespace coco {

/// @brief USB device as seen by the host.
/// Connects itself to an actual USB device when it is plugged in and the filter on the device descriptor returns
/// true.
class UsbHostDevice_io_uring : public Device, public Loop_io_uring::CompletionHandler, public coco::IntrusiveListNode {
public:
    /// @brief Constructor.
    /// @param loop Event loop
    UsbHostDevice_io_uring(Loop_io_uring &loop);

    ~UsbHostDevice_io_uring() override;

    //void getDescriptor(usb::DescriptorType type, void *data, int &size) override;

    /// @brief Open the device.
    /// @param path Path to device, e.g. obtained from UsbMonitor
    /// @return true when successful
    bool open(const std::filesystem::path &path);
    void close() override;


    /// @brief Buffer for control transfers
    /// Max size is 512 according to spec: https://www.techdesignforums.com/practice/technique/usb-3-0-protocol-layer-2/
    class ControlBuffer : public coco::Buffer, public coco::IntrusiveListNode  {
        friend class UsbHostDevice_io_uring;
    public:
        ControlBuffer(int capacity, UsbHostDevice_io_uring &device);
        ~ControlBuffer() override;

        // Buffer methods
        bool start() override;
        bool cancel() override;

    protected:
        bool transfer();
        //void onCompletion(io_uring_cqe &cqe, int id) override;

        usb::Setup setup_;
        UsbHostDevice_io_uring &device_;
    };


    class Endpoint;

    /// @brief Buffer for transferring data to/from an endpoint
    ///
    class Buffer : public coco::Buffer, public coco::IntrusiveListNode {
        friend class UsbHostDevice_io_uring;
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
        friend class UsbHostDevice_io_uring;
        friend class Buffer;
    public:
        Endpoint(UsbHostDevice_io_uring &device, int inAddress, int outAddress);
        Endpoint(UsbHostDevice_io_uring &device, int address) : Endpoint(device,  usb::IN | address, usb::OUT | address) {}
        ~Endpoint();

        // BufferDevice methods
        int getBufferCount() override;
        Buffer &getBuffer(int index) override;

    protected:
        UsbHostDevice_io_uring &device_;
        int inAddress_;
        int outAddress_;

        // list of buffers
        IntrusiveList<Buffer> buffers_;
    };

protected:
    void onCompletion(io_uring_cqe &cqe, int id) override;

    Loop_io_uring &loop_;

    // device handle
    static constexpr int INVALID_HANDLE_VALUE = -1; 
    int handle_ = INVALID_HANDLE_VALUE;

    // list of all buffers for control endpoint
    IntrusiveList<ControlBuffer> controlBuffers_;

    // list of all bulk/interrupt endpoints
    IntrusiveList<Endpoint> endpoints_;
};

} // namespace coco

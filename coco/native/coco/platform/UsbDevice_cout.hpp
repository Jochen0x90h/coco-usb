#pragma once

#include <coco/UsbDevice.hpp>
#include <coco/BufferDevice.hpp>
#include <coco/platform/Loop_native.hpp>


namespace coco {

/// @brief Implementation of an USB device that simply writes info about the transfer operations to std::cout
///
class UsbDevice_cout : public UsbDevice {
public:
    /// @brief Constructor
    /// @param getDescriptor callback for obtaining descriptors
    /// @param onSetConfiguration callback for setting the configuration (libusb_set_configuration() on host), always called from event loop
    /// @param onRequest callback for vendor specific request
    UsbDevice_cout(Loop_native &loop);

    // UsbDevice methods
    usb::Setup getSetup() override;
    void acknowledge() override;
    void stall() override;


    /// @brief Buffer for control transfers
    ///
    class ControlBuffer : public coco::Buffer, public IntrusiveListNode, public IntrusiveListNode2 {
        friend class UsbDevice_cout;
    public:
        ControlBuffer(int capacity, UsbDevice_cout &device);
        ~ControlBuffer() override;

        // Buffer methods
        bool start(Op op) override;
        bool cancel() override;

    protected:
        UsbDevice_cout &device_;
    };


    class Endpoint;

    /// @brief Buffer for transferring data to/from an endpoint
    ///
    class Buffer : public coco::Buffer, public IntrusiveListNode, public IntrusiveListNode2 {
        friend class UsbDevice_cout;
    public:
        Buffer(int capacity, Endpoint &endpoint);
        ~Buffer() override;

        // Buffer methods
        bool start(Op op) override;
        bool cancel() override;

    protected:
        Endpoint &endpoint_;
    };

    /// @brief Endpoint
    ///
    class Endpoint : public BufferDevice, public IntrusiveListNode {
        friend class UsbDevice_cout;
        friend class BulkBuffer;
    public:
        Endpoint(UsbDevice_cout &device, int index);
        ~Endpoint();

        // BufferDevice methods
        int getBufferCount() override;
        Buffer &getBuffer(int index) override;

    protected:
        UsbDevice_cout &device_;
        int index_;

        // list of buffers
        IntrusiveList<Buffer> buffers_;
    };

protected:
    Coroutine readDescriptors();
    void handle();

    Loop_native &loop_;
    TimedTask<Callback> callback_;

    usb::Setup setup_;

    // binary/text extracted from the device descriptor
    bool text_;

    // list of control buffers
    IntrusiveList<ControlBuffer> controlBuffers_;

    // list of bulk/interrupt endpoints
    IntrusiveList<Endpoint> endpoints_;

    // list of active transfers
    IntrusiveList2<ControlBuffer> controlTransfers_;
    IntrusiveList2<Buffer> transfers_;
};

} // namespace coco

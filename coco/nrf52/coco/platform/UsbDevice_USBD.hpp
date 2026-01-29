#pragma once

#include <coco/UsbDevice.hpp>
#include <coco/BufferDevice.hpp>
#include <coco/InterruptQueue.hpp>
#include <coco/platform/Loop_Queue.hpp>
#include <coco/platform/platform.hpp>
#include <coco/platform/nvic.hpp>


namespace coco {

/// @brief Implementation of USB device for nRF52 using USBD peripheral
/// https://docs.nordicsemi.com/bundle/ps_nrf52840/page/usbd.html
///
/// Resources:
///   NRF_USBD
class UsbDevice_USBD : public UsbDevice, public Loop_Queue::Handler {
public:
    /// @brief Constructor
    /// @param loop event loop
    UsbDevice_USBD(Loop_Queue &loop);
    ~UsbDevice_USBD() override;

    // UsbDevice methods
    usb::Setup getSetup() override;
    void acknowledge() override;
    void stall() override;


    // internal control buffer base class, derives from IntrusiveListNode for the list of buffers and Loop_Queue::Handler to be notified from the event loop
    class ControlBufferBase : public Buffer, public IntrusiveListNode, public Loop_Queue::Handler {
        friend class UsbDevice_USBD;
    public:
        static constexpr int BUFFER_SIZE = 64;

        ControlBufferBase(uint8_t *data, int capacity, UsbDevice_USBD &device);
        ~ControlBufferBase() override;

        // Buffer methods
        bool start(Op op) override;
        bool cancel() override;

    protected:
        bool readNext();
        void writeFirst();
        bool writeNext();
        void handle() override;

        UsbDevice_USBD &device_;
        Op op_;
        uint8_t *transferIt_;
        uint8_t *transferEnd_;
    };

    /// @brief Buffer for control transfers
    /// @tparam N size of buffer
    template <int N>
    class ControlBuffer : public ControlBufferBase {
    public:
        ControlBuffer(UsbDevice_USBD &device) : ControlBufferBase(buffer, N, device) {}

    protected:
        alignas(4) uint8_t buffer[N];
    };


    class Endpoint;

    // internal bulk buffer base class, derives from IntrusiveListNode for the list of buffers and Loop_Queue::Handler to be notified from the event loop
    class BufferBase : public Buffer, public IntrusiveListNode, public Loop_Queue::Handler {
        friend class UsbDevice_USBD;
    public:
        static constexpr int BUFFER_SIZE = 64;

        BufferBase(uint8_t *data, int capacity, Endpoint &endpoint);
        ~BufferBase() override;

        // Buffer methods
        bool start(Op op) override;
        bool cancel() override;

    protected:
        bool readNext();
        void writeFirst();
        bool writeNext();
        void handle() override;

        Endpoint &endpoint_;
        Op op_;
        uint8_t *transferIt_;
        uint8_t *transferEnd_;
    };

    /// @brief Bulk/Interrupt Endpoint
    ///
    class Endpoint : public BufferDevice, public IntrusiveListNode {
        friend class UsbDevice_USBD;
        friend class UsbDevice_USBD::BufferBase;
    public:
        Endpoint(UsbDevice_USBD &device, int inIndex, int outIndex);
        Endpoint(UsbDevice_USBD &device, int index) : Endpoint(device, index, index) {}
        ~Endpoint() override;

        // BufferDevice methods
        int getBufferCount() override;
        BufferBase &getBuffer(int index) override;

    protected:
        UsbDevice_USBD &device_;
        int inIndex_;
        int outIndex_;

        // list of buffers
        IntrusiveList<BufferBase> buffers_;
    };

    /// @brief Buffer for bulk transfers on an endpoint
    /// @tparam N size of buffer
    template <int N>
    class Buffer : public BufferBase {
    public:
        Buffer(Endpoint &endpoint) : BufferBase(buffer, N, endpoint) {}

    protected:
        alignas(4) uint8_t buffer[N];
    };

    /// @brief USB interrupt handler, needs to be called from USBD_IRQHandler() (check in startup code in coco if interrupt handler exists!)
    ///
    void USBD_IRQHandler();
protected:
    void handle() override;

    Loop_Queue &loop_;

    // state and events for interrupt handler
    std::atomic<State> iState_ = State::OPENING; // device is in opening state until it gets configured by the host
    std::atomic<Events> iEvents_ = Events::NONE;

    // list of control buffers
    IntrusiveList<ControlBufferBase> controlBuffers_;

    // control transfer mode
    enum class Mode : uint8_t {
        IDLE,

        // data stage of control transfer
        DATA_IN,
        DATA_OUT,
    };
    Mode controlMode_ = Mode::IDLE;

    // active control transfers
    InterruptQueue<ControlBufferBase> controlTransfers_;

    // list of bulk/interrupt endpoints
    IntrusiveList<Endpoint> endpoints_;

    // for each endpont a queue of active in and out transfers
    struct Transfer {
        InterruptQueue<BufferBase> in;
        bool outAvailable;
        InterruptQueue<BufferBase> out;
    };
    Transfer bulkTransfers_[7];
};

} // namespace coco

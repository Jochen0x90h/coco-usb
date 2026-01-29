#pragma once

#include <coco/UsbDevice.hpp>
#include <coco/BufferDevice.hpp>
#include <coco/InterruptQueue.hpp>
#include <coco/platform/Loop_Queue.hpp>
#include <coco/platform/platform.hpp>
#include <coco/platform/usbd.hpp>
#include <coco/platform/nvic.hpp>


// not all devices implement USB
#ifdef HAVE_USBD

namespace coco {

/// @brief Implementation of USB device on stm32 using USB peripheral
///
/// Resources:
///   USB
///   USB PMA (packet memory area)
class UsbDevice_USB : public UsbDevice, public Loop_Queue::Handler {
public:
    /// @brief Constructor
    /// @param loop Event loop
    UsbDevice_USB(Loop_Queue &loop);
    ~UsbDevice_USB() override;

    // UsbDevice methods
    usb::Setup getSetup() override;
    void acknowledge() override;
    void stall() override;


    // internal control buffer base class, derives from IntrusiveListNode for the list of buffers and Loop_Queue::Handler to be notified from the event loop
    class ControlBufferBase : public Buffer, public IntrusiveListNode, public Loop_Queue::Handler {
        friend class UsbDevice_USB;
    public:
        static constexpr int BUFFER_SIZE = 64;

        ControlBufferBase(uint8_t *data, int capacity, UsbDevice_USB &device);
        ~ControlBufferBase() override;

        // Buffer methods
        bool start(Op op) override;
        bool cancel() override;

    protected:
        bool readNext();
        void writeFirst();
        bool writeNext();
        void handle() override;

        UsbDevice_USB &device_;
        Op op_;
        uint8_t *transferIt_;
        uint8_t *transferEnd_;
    };

    /// @brief Buffer for control transfers
    /// @tparam N Size of buffer
    template <int N>
    class ControlBuffer : public ControlBufferBase {
    public:
        ControlBuffer(UsbDevice_USB &device) : ControlBufferBase(buffer, N, device) {}

    protected:
        alignas(4) uint8_t buffer[N];
    };


    class Endpoint;

    // internal bulk buffer base class, derives from IntrusiveListNode for the list of buffers and Loop_Queue::Handler to be notified from the event loop
    class BufferBase : public Buffer, public IntrusiveListNode, public Loop_Queue::Handler {
        friend class UsbDevice_USB;
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
        friend class UsbDevice_USB;
        friend class UsbDevice_USB::BufferBase;
    public:
        /// @brief Constructor
        /// @param device Reference to USB device
        /// @param inIndex Index of IN endpoint, range [1, 7] or 0 when not used
        /// @param outIndex Index of OUT endpoint, range [1, 7] or 0 when not used
        /// @param type Type, BULK or INTERRUPT
        Endpoint(UsbDevice_USB &device, int inIndex, int outIndex, usbd::EndpointType type = usbd::EndpointType::BULK);

        /// @brief Constructor
        /// @param device Reference to USB device
        /// @param index Index of IN and OUT endpoints, range [1, 7]
        /// @param type Type, BULK or INTERRUPT
        Endpoint(UsbDevice_USB &device, int index, usbd::EndpointType type = usbd::EndpointType::BULK) : Endpoint(device, index, index, type) {}
        ~Endpoint() override;

        // BufferDevice methods
        int getBufferCount() override;
        BufferBase &getBuffer(int index) override;

    protected:
        UsbDevice_USB &device_;
        int inIndex_;
        int outIndex_;
        usbd::EndpointType type_;

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

    /// @brief USB interrupt handler, needs to be called from USB_IRQHandler() or USB_LP_IRQHandler() (check in startup code in coco if interrupt handler exists!)
    ///
    void USB_IRQHandler();
protected:
    void handle() override;

    Loop_Queue &loop_;

    // state and events for interrupt handler
    std::atomic<State> iState_ = State::OPENING; // device is in opening state until it gets configured by the host (also see constructors)
    std::atomic<Events> iEvents_ = Events::NONE;

    // list of control buffers
    IntrusiveList<ControlBufferBase> controlBuffers_;

    // control transfer mode
    enum class Mode : uint8_t {
        IDLE,

        // data stage of control transfer
        DATA_IN,
        DATA_OUT,

        // set USB address (and status stage of control OUT transfer)
        SET_ADDRESS,

        // status stage of control transfer
        STATUS,
    };
    Mode controlMode_ = Mode::IDLE;

    // queue of active control transfers
    InterruptQueue<ControlBufferBase> controlTransfers_;


    // list of bulk/interrupt endpoints
    IntrusiveList<Endpoint> endpoints_;

    // for each endpont a queue of active in and out transfers (bulkTransfers[0].in is for IN endpoint 1)
    struct Transfer {
        InterruptQueue<BufferBase> in;
        bool outAvailable;
        InterruptQueue<BufferBase> out;
    };
    Transfer transfers_[7];
};

} // namespace coco

#endif // HAVE_USBD

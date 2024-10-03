#pragma once

#include <coco/UsbDevice.hpp>
#include <coco/BufferDevice.hpp>
#include <coco/platform/Loop_Queue.hpp>
#include <coco/platform/platform.hpp>
#include <coco/platform/nvic.hpp>


// not all devices implement USB
#ifdef USB
namespace coco {

/**
 * Implementation of USB device on stm32 using USB peripheral
 *
 * Reference manual:
 *   f0:
 *     https://www.st.com/resource/en/reference_manual/dm00031936-stm32f0x1stm32f0x2stm32f0x8-advanced-armbased-32bit-mcus-stmicroelectronics.pdf
 *       USB: Section 30
 *   g4:
 *     https://www.st.com/resource/en/reference_manual/rm0440-stm32g4-series-advanced-armbased-32bit-mcus-stmicroelectronics.pdf
 *       USB: Section 45
 * Resources:
 *   USB
 */
class UsbDevice_USB : public UsbDevice, public Loop_Queue::Handler {
public:
	enum class EndpointType {
		BULK = USB_EP_BULK,
		INTERRUPT = USB_EP_INTERRUPT
	};

	/**
	 * Constructor
	 * @param loop event loop
	 */
	UsbDevice_USB(Loop_Queue &loop);
	~UsbDevice_USB() override;

	// Device methods
	//StateTasks<const State, Events> &getStateTasks() override;
	//State state() override;
	//bool ready() {return this->stat == State::READY;}
	//[[nodiscard]] Awaitable<Condition> until(Condition condition) override;

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
		bool finishRead();
		void startWrite();
		bool finishWrite();
		void handle() override;

		UsbDevice_USB &device;

		Op op;
		uint8_t *transferIt;
		uint8_t *transferEnd;
	};

	/**
	 * Buffer for control transfers
	 * @tparam N Size of buffer
	 */
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
		bool finishRead();
		void startWrite();
		bool finishWrite();
		void handle() override;


		Endpoint &endpoint;

		Op op;
		uint8_t *transferIt;
		uint8_t *transferEnd;
	};

	/**
	 * Bulk/Interrupt Endpoint
	 */
	class Endpoint : public BufferDevice, public IntrusiveListNode {
		friend class UsbDevice_USB;
		friend class UsbDevice_USB::BufferBase;
	public:
		/**
		 * Constructor
		 * @param device Reference to USB device
		 * @param inIndex Index of IN endpoint, 1 - 7
		 * @param outIndex Index of OUT endpoint, 1 - 7
		 * @param type Type, BULK or INTERRUPT
		 */
		Endpoint(UsbDevice_USB &device, int inIndex, int outIndex, EndpointType type = EndpointType::BULK);

		/**
		 * Constructor
		 * @param device Reference to USB device
		 * @param index Index of IN and OUT endpoints, 1 - 7
		 * @param type Type, BULK or INTERRUPT
		 */
		Endpoint(UsbDevice_USB &device, int index, EndpointType type = EndpointType::BULK) : Endpoint(device, index, index, type) {}
		~Endpoint() override;

		// Device methods
		//StateTasks<const State, Events> &getStateTasks() override;
		//State state() override;
		//[[nodiscard]] Awaitable<Condition> until(Condition condition) override;

		// BufferDevice methods
		int getBufferCount() override;
		BufferBase &getBuffer(int index) override;

	protected:
		UsbDevice_USB &device;
		int inIndex;
		int outIndex;
		EndpointType type;

		// list of buffers
		IntrusiveList<BufferBase> buffers;
	};

	/**
	 * Buffer for bulk transfers on an endpoint
	 * @tparam N size of buffer
	 */
	template <int N>
	class Buffer : public BufferBase {
	public:
		Buffer(Endpoint &endpoint) : BufferBase(buffer, N, endpoint) {}

	protected:
		alignas(4) uint8_t buffer[N];
	};

	/**
	 * USB interrupt handler, needs to be called from USB_IRQHandler() or USB_LP_IRQHandler() (check in startup code in coco if interrupt handler exists!)
	 */
	void USB_IRQHandler();
protected:
	void handle() override;

	Loop_Queue &loop;

	// state
	//StateTasks<State, Events> st = State::OPENING;
	std::atomic<State> stat = State::OPENING; // device is in opening state until it gets configured by the host (also see constructors)
	//CoroutineTaskList<Condition> stateTasks;

	// events
	std::atomic<Events> events = Events::NONE;
	//std::atomic<Condition> condition = Condition::NONE;

	// list of control buffers
	IntrusiveList<ControlBufferBase> controlBuffers;

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
	Mode controlMode = Mode::IDLE;

	// queue of active control transfers
	InterruptQueue<ControlBufferBase> controlTransfers;


	// list of bulk/interrupt endpoints
	IntrusiveList<Endpoint> endpoints;

	// for each endpont a queue of active in and out transfers (bulkTransfers[0].in is for IN endpoint 1)
	struct Transfer {
		InterruptQueue<BufferBase> in;
		bool outAvailable;
		InterruptQueue<BufferBase> out;
	};
	Transfer transfers[7];
};

} // namespace coco
#endif

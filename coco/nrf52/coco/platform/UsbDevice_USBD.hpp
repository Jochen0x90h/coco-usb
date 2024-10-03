#pragma once

#include <coco/UsbDevice.hpp>
#include <coco/BufferDevice.hpp>
#include <coco/platform/Loop_Queue.hpp>
#include <coco/platform/platform.hpp>
#include <coco/platform/nvic.hpp>


namespace coco {

/**
 * Implementation of USB device for nRF52 using USBD peripheral
 * https://infocenter.nordicsemi.com/index.jsp?topic=%2Fps_nrf52840%2Fusbd.html&cp=5_0_0_5_34
 *
 * Resources:
 * NRF_USBD
 */
class UsbDevice_USBD : public UsbDevice, public Loop_Queue::Handler {
public:
	/**
	 * Constructor
	 * @param loop event loop
	 */
	UsbDevice_USBD(Loop_Queue &loop);
	~UsbDevice_USBD() override;

	// Device methods
	//StateTasks<const State, Events> &getStateTasks() override;
	//State state() override;
	//bool ready() {return this->stat == State::READY;}
	//[[nodiscard]] Awaitable<Condition> until(Condition condition) override;

	// UsbDevice methods
	//[[nodiscard]] Awaitable<usb::Setup *> request(usb::Setup &setup) override;
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
		bool finishRead();
		void startWrite();
		bool finishWrite();
		void handle() override;

		UsbDevice_USBD &device;

		Op op;
		uint8_t *transferIt;
		uint8_t *transferEnd;
	};

	/**
	 * Buffer for control transfers
	 * @tparam N size of buffer
	 */
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
		friend class UsbDevice_USBD;
		friend class UsbDevice_USBD::BufferBase;
	public:
		Endpoint(UsbDevice_USBD &device, int inIndex, int outIndex);
		Endpoint(UsbDevice_USBD &device, int index) : Endpoint(device, index, index) {}
		~Endpoint() override;

		// Device methods
		//StateTasks<const State, Events> &getStateTasks() override;
		//State state() override;
		//[[nodiscard]] Awaitable<Condition> until(Condition condition) override;

		// BufferDevice methods
		int getBufferCount() override;
		BufferBase &getBuffer(int index) override;

	protected:
		UsbDevice_USBD &device;
		int inIndex;
		int outIndex;

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
		USB interrupt handler, needs to be called from USBD_IRQHandler() (check in startup code in coco if interrupt handler exists!)
	*/
	void USBD_IRQHandler();
protected:
	void handle() override;

	Loop_Queue &loop;

	// state
	//StateTasks<State, Events> st = State::OPENING;
	std::atomic<State> stat = State::OPENING; // device is in opening state until it gets configured by the host
	//CoroutineTaskList<Condition> stateTasks;

	// events
	std::atomic<Events> events = Events::NONE;

	// list of control buffers
	IntrusiveList<ControlBufferBase> controlBuffers;

	// control transfer mode
	enum class Mode : uint8_t {
		IDLE,

		// data stage of control transfer
		DATA_IN,
		DATA_OUT,
	};
	Mode controlMode = Mode::IDLE;

	// active control transfers
	InterruptQueue<ControlBufferBase> controlTransfers;

	// list of bulk/interrupt endpoints
	IntrusiveList<Endpoint> endpoints;

	// for each endpont a queue of active in and out transfers
	struct Transfer {
		InterruptQueue<BufferBase> in;
		bool outAvailable;
		InterruptQueue<BufferBase> out;
	};
	Transfer bulkTransfers[7];
};

} // namespace coco

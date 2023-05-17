#pragma once

#include <coco/UsbDevice.hpp>
#include <coco/BufferImpl.hpp>
#include <coco/platform/platform.hpp>
#include <coco/platform/Loop_RTC0.hpp>


namespace coco {

/**
	Implementation of USB device for nRF52 using USBD peripheral
	https://infocenter.nordicsemi.com/index.jsp?topic=%2Fps_nrf52840%2Fusbd.html&cp=5_0_0_5_34

	Resources:
	NRF_USBD
*/
class UsbDevice_USBD : public UsbDevice, public Loop_RTC0::Handler {
public:
	UsbDevice_USBD(Loop_RTC0 &loop);
	~UsbDevice_USBD() override;

	State state() override;
	bool ready() {return this->stat == State::READY;}
	[[nodiscard]] Awaitable<State> untilState(State state) override;

	[[nodiscard]] Awaitable<usb::Setup *> request(usb::Setup &setup) override;

	void acknowledge() override;
	void stall() override;


	class ControlBufferBase : public LinkedListNode, public LinkedListNode2, public BufferImpl {
		friend class UsbDevice_USBD;
	public:
		static constexpr int BUFFER_SIZE = 64;

		ControlBufferBase(uint8_t *data, int size, UsbDevice_USBD &device);
		~ControlBufferBase() override;

		bool startInternal(int size, Op op) override;
		void cancel() override;

	protected:
		void finishRead();
		void startWrite();
		void finishWrite();

		UsbDevice_USBD &device;

		uint8_t *transferIt;
		uint8_t *transferEnd;
		bool partial;
		bool writing;
	};

	/**
		Buffer for control transfers
		@tparam N size of buffer
	*/
	template <int N>
	class ControlBuffer : public ControlBufferBase {
	public:
		ControlBuffer(UsbDevice_USBD &device) : ControlBufferBase(buffer, N, device) {}

	protected:
		alignas(4) uint8_t buffer[N];
	};


	class BulkEndpoint;

	class BulkBufferBase : public LinkedListNode, public LinkedListNode2, public BufferImpl {
		friend class UsbDevice_USBD;
	public:
		static constexpr int BUFFER_SIZE = 64;

		BulkBufferBase(uint8_t *data, int size, BulkEndpoint &endpoint);
		~BulkBufferBase() override;

		bool startInternal(int size, Op op) override;
		void cancel() override;

	protected:
		void finishRead();
		void startWrite();

		// finish write operation, return false if more packets need to be transferred
		bool finishWrite();


		BulkEndpoint &endpoint;

		uint8_t *transferIt;
		uint8_t *transferEnd;
		bool partial;
		bool writing;
	};

	/**
		BulkEndpoint
	*/
	class BulkEndpoint : public LinkedListNode, public Device {
		friend class UsbDevice_USBD;
		friend class UsbDevice_USBD::BulkBufferBase;
	public:
		BulkEndpoint(UsbDevice_USBD &device, int inIndex, int outIndex);
		BulkEndpoint(UsbDevice_USBD &device, int index) : BulkEndpoint(device, index, index) {}
		~BulkEndpoint();

		State state() override;
		Awaitable<State> untilState(State state) override;
		int getBufferCount() override;
		BulkBufferBase &getBuffer(int index) override;

	protected:
		UsbDevice_USBD &device;
		int inIndex;
		int outIndex;

		// list of buffers
		LinkedList<BulkBufferBase> buffers;
	};

	/**
		Buffer for bulk transfers on an endpoint
		@tparam N size of buffer
	*/
	template <int N>
	class BulkBuffer : public BulkBufferBase {
	public:
		BulkBuffer(BulkEndpoint &endpoint) : BulkBufferBase(buffer, N, endpoint) {}

	protected:
		alignas(4) uint8_t buffer[N];
	};

protected:
	void handle() override;

	// state
	State stat = State::DISABLED;
	TaskList<State> stateTasks;

	// coroutines waiting for a control request
	TaskList<usb::Setup *> requestTasks;

	// list of control buffers
	LinkedList<ControlBufferBase> controlBuffers;

	// active control transfers
	LinkedList2<ControlBufferBase> controlTransfers;

	// list of bulk endpoints
	LinkedList<BulkEndpoint> bulkEndpoints;

	// status flags
	int inBusy = 0; // flags: 0x1 << index
	int outAvailable = 0; // flags: 0x10000 << index

	// for each endpont a list of active in and out transfers
	LinkedList2<BulkBufferBase> inTransfers[7];
	LinkedList2<BulkBufferBase> outTransfers[7];
};

} // namespace coco

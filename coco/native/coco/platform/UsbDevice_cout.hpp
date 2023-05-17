#pragma once

#include <coco/UsbDevice.hpp>
#include <coco/BufferImpl.hpp>
#include <coco/platform/Loop_native.hpp>


namespace coco {

/**
	Implementation of an USB device that simply writes info about the transfer operations to std::cout
*/
class UsbDevice_cout : public UsbDevice, public Loop_native::YieldHandler {
public:
	/**
		Constructor
		@param getDescriptor callback for obtaining descriptors
		@param onSetConfiguration callback for setting the configuration (libusb_set_configuration() on host), always called from event loop
		@param onRequest callback for vendor specific request
	*/
	UsbDevice_cout(Loop_native &loop);

	State state() override;
	bool ready() {return this->stat == State::READY;}
	[[nodiscard]] Awaitable<State> untilState(State state) override;

	[[nodiscard]] Awaitable<usb::Setup *> request(usb::Setup &setup) override;

	void acknowledge() override;
	void stall() override;


	/**
		Buffer for control transfers
	*/
	class ControlBuffer : public LinkedListNode, public LinkedListNode2, public BufferImpl {
		friend class UsbDevice_cout;
	public:
		ControlBuffer(UsbDevice_cout &device, int size);
		~ControlBuffer() override;

		bool startInternal(int size, Op op) override;
		void cancel() override;

	protected:
		UsbDevice_cout &device;
	};


	class BulkEndpoint;

	/**
		Buffer for transferring data to/from an endpoint
	*/
	class BulkBuffer : public LinkedListNode, public LinkedListNode2, public BufferImpl {
		friend class UsbDevice_cout;
	public:
		BulkBuffer(BulkEndpoint &endpoint, int size);
		~BulkBuffer() override;

		bool startInternal(int size, Op op) override;
		void cancel() override;

	protected:
		BulkEndpoint &endpoint;
	};

	/**
		BulkEndpoint
	*/
	class BulkEndpoint : public LinkedListNode, public coco::Device {
		friend class UsbDevice_cout;
		friend class BulkBuffer;
	public:
		BulkEndpoint(UsbDevice_cout &device, int index);
		~BulkEndpoint();

		State state() override;
		Awaitable<State> untilState(State state) override;
		int getBufferCount() override;
		BulkBuffer &getBuffer(int index) override;

	protected:
		UsbDevice_cout &device;
		int index;

		// list of buffers
		LinkedList<BulkBuffer> buffers;
	};

protected:
	void handle() override;

	Loop_native &loop;

	// state and coroutines waiting for a state
	State stat = State::DISABLED;
	TaskList<State> stateTasks;

	// coroutines waiting for a control request
	TaskList<usb::Setup *> requestTasks;

	bool readDescriptor = true;

	// binary/text extracted from the device descriptor
	bool text;

	// list of control buffers
	LinkedList<ControlBuffer> controlBuffers;

	// list of bulk endpoints
	LinkedList<BulkEndpoint> bulkEndpoints;

	// list of active transfers
	LinkedList2<ControlBuffer> controlTransfers;
	LinkedList2<BulkBuffer> bulkTransfers;
};

} // namespace coco

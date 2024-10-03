#pragma once

#include <coco/UsbDevice.hpp>
#include <coco/BufferDevice.hpp>
#include <coco/platform/Loop_native.hpp>


namespace coco {

/**
	Implementation of an USB device that simply writes info about the transfer operations to std::cout
*/
class UsbDevice_cout : public UsbDevice {
public:
	/**
		Constructor
		@param getDescriptor callback for obtaining descriptors
		@param onSetConfiguration callback for setting the configuration (libusb_set_configuration() on host), always called from event loop
		@param onRequest callback for vendor specific request
	*/
	UsbDevice_cout(Loop_native &loop);

	// Device methods
	//StateTasks<const State, Events> &getStateTasks() override;
	//State state() override;
	//bool ready() {return this->stat == State::READY;}
	//[[nodiscard]] Awaitable<Condition> until(Condition condition) override;

	// UsbDevice methods
	usb::Setup getSetup() override;
	void acknowledge() override;
	void stall() override;


	/**
		Buffer for control transfers
	*/
	class ControlBuffer : public coco::Buffer, public IntrusiveListNode, public IntrusiveListNode2 {
		friend class UsbDevice_cout;
	public:
		ControlBuffer(int capacity, UsbDevice_cout &device);
		~ControlBuffer() override;

		// Buffer methods
		bool start(Op op) override;
		bool cancel() override;

	protected:
		UsbDevice_cout &device;
	};


	class Endpoint;

	/**
	 * Buffer for transferring data to/from an endpoint
	 */
	class Buffer : public coco::Buffer, public IntrusiveListNode, public IntrusiveListNode2 {
		friend class UsbDevice_cout;
	public:
		Buffer(int capacity, Endpoint &endpoint);
		~Buffer() override;

		// Buffer methods
		bool start(Op op) override;
		bool cancel() override;

	protected:
		Endpoint &endpoint;
	};

	/**
	 * Endpoint
	 */
	class Endpoint : public BufferDevice, public IntrusiveListNode {
		friend class UsbDevice_cout;
		friend class BulkBuffer;
	public:
		Endpoint(UsbDevice_cout &device, int index);
		~Endpoint();

		// Device methods
		//StateTasks<const State, Events> &getStateTasks() override;
		//State state() override;
		//[[nodiscard]] Awaitable<Condition> until(Condition condition) override;

		// BufferDevice methods
		int getBufferCount() override;
		Buffer &getBuffer(int index) override;

	protected:
		UsbDevice_cout &device;
		int index;

		// list of buffers
		IntrusiveList<Buffer> buffers;
	};

protected:
	void handle();

	Loop_native &loop;
	TimedTask<Callback> callback;

	// state
	//State stat = State::OPENING;//State::DISABLED;
	//CoroutineTaskList<Condition> stateTasks;
	//StateTasks<State, Events> st = State::OPENING;

	bool readDescriptor = true;

	// binary/text extracted from the device descriptor
	bool text;

	// list of control buffers
	IntrusiveList<ControlBuffer> controlBuffers;

	// list of bulk/interrupt endpoints
	IntrusiveList<Endpoint> endpoints;

	// list of active transfers
	IntrusiveList2<ControlBuffer> controlTransfers;
	IntrusiveList2<Buffer> transfers;
};

} // namespace coco

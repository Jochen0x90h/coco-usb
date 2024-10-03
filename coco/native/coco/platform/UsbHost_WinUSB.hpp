#pragma once

#include <coco/usb.hpp>
#include <coco/BufferDevice.hpp>
#include <coco/platform/Loop_Win32.hpp> // includes Windows.h
#include <winusb.h>
#include <string>
#include <map>
#include <functional>


namespace coco {

/**
	USB host implementation using Win32 and WinUsb
	https://learn.microsoft.com/en-us/windows/win32/api/winusb/
*/
class UsbHost_WinUSB {
public:
	UsbHost_WinUSB(Loop_Win32 &loop);
	//~UsbHost_WinUSB() override;


	class Device;

	/**
		Buffer for control transfers
		Max size is 512 according to spec: https://www.techdesignforums.com/practice/technique/usb-3-0-protocol-layer-2/
	*/
	class ControlBuffer : public coco::Buffer, public IntrusiveListNode, public IntrusiveListNode2 {
		friend class Device;
	public:
		ControlBuffer(int capacity, Device &device);
		~ControlBuffer() override;

		// Buffer methods
		bool start(Op op) override;
		bool cancel() override;

	protected:
		void start();
		void handle(OVERLAPPED *overlapped);

		Device &device;
		OVERLAPPED overlapped;
	};


	class Endpoint;

	/**
		Buffer for transferring data to/from an endpoint
	*/
	class Buffer : public coco::Buffer, public IntrusiveListNode, public IntrusiveListNode2 {
		friend class UsbHost_WinUSB::Device;
	public:
		Buffer(int capacity, Endpoint &endpoint);
		~Buffer() override;

		// Buffer methods
		bool start(Op op) override;
		bool cancel() override;

	protected:
		void start();
		void handle(OVERLAPPED *overlapped);

		Endpoint &endpoint;

		Op op;
		OVERLAPPED overlapped[2];
		int index;
	};

	/**
	 * Bulk/Interrupt endpoint
	 */
	class Endpoint : public BufferDevice, public IntrusiveListNode {
		friend class UsbHost_WinUSB::Device;
		friend class Buffer;
	public:
		Endpoint(UsbHost_WinUSB::Device &device, int inAddress, int outAddress);
		Endpoint(UsbHost_WinUSB::Device &device, int address) : Endpoint(device,  usb::IN | address, usb::OUT | address) {}
		~Endpoint();

		// Device methods
		//StateTasks<const State, Events> &getStateTasks() override;
		//State state() override;
		//[[nodiscard]] Awaitable<Condition> until(Condition condition) override;

		// BufferDevice methods
		int getBufferCount() override;
		Buffer &getBuffer(int index) override;

	protected:
		UsbHost_WinUSB::Device &device;
		int inAddress;
		int outAddress;

		// list of buffers
		IntrusiveList<Buffer> buffers;
	};

	/**
		USB device as seen by the host. Connects itself to an actual USB device when it is plugged in and the
		filter on the device descriptor returns true.
	*/
	class Device : public coco::Device, public Loop_Win32::CompletionHandler, public IntrusiveListNode {
		friend class UsbHost_WinUSB;
	public:

		/**
			Constructor
			@param host usb host
			@param filter filter to indicate if an usb device is handled by this device instance
		*/
		Device(UsbHost_WinUSB &host, std::function<bool (const usb::DeviceDescriptor &)> filter);

		~Device() override;

		// Device methods
		//StateTasks<const State, Events> &getStateTasks() override;
		//State state() override;
		//bool ready() {return this->stat == State::READY;}
		//[[nodiscard]] Awaitable<Condition> until(Condition condition) override;

		//void getDescriptor(usb::DescriptorType type, void *data, int &size) override;

	protected:
		void connect(HANDLE file, void *interface);
		void disconnect();
		void handle(OVERLAPPED *overlapped) override;

		UsbHost_WinUSB &host;
		std::function<bool (const usb::DeviceDescriptor &)> filter;

		// iterator of device list and flag for remove detection
		std::map<std::string, Device *>::iterator it;
		bool flag;

		// file handle and WinUsb interface
		HANDLE file = INVALID_HANDLE_VALUE;
		WINUSB_INTERFACE_HANDLE interface = nullptr;

		// state
		//State stat = State::OPENING;//DISABLED;
		//CoroutineTaskList<Condition> stateTasks;
		//StateTasks<State, Events> st = State::OPENING;

		// list of all buffers for control endpoint
		IntrusiveList<ControlBuffer> controlBuffers;

		// list of all bulk/interrupt endpoints
		IntrusiveList<Endpoint> endpoints;

		// pending transfers (needed to queue transfers when device is in OPENING or PAUSE state)
		IntrusiveList2<ControlBuffer> controlTransfers;
		IntrusiveList2<Buffer> transfers;
	};

protected:
	void handle();

	Loop_Win32 &loop;
	TimedTask<Callback> callback;
	std::map<std::string, Device *> deviceInfos;
	IntrusiveList<Device> devices;
};

} // namespace coco

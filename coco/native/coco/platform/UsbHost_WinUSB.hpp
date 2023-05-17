#pragma once

#include <coco/usb.hpp>
#include <coco/UsbHostDevice.hpp>
#include <coco/BufferImpl.hpp>
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
class UsbHost_WinUSB : public Loop_Win32::TimeHandler {
public:
	UsbHost_WinUSB(Loop_Win32 &loop);
	~UsbHost_WinUSB() override;

	void handle() override;

	class Device;

	/**
		Buffer for control transfers
		Max size is 512 according to spec: https://www.techdesignforums.com/practice/technique/usb-3-0-protocol-layer-2/
	*/
	class ControlBuffer : public LinkedListNode, public BufferImpl {
		friend class Device;
	public:
		ControlBuffer(Device &device, int size);
		~ControlBuffer() override;

		bool setHeader(const uint8_t *data, int size) override;
		using BufferImpl::setHeader;
		//void setSetup(const usb::Setup &setup) override;
		bool startInternal(int size, Op op) override;
		void cancel() override;

	protected:
		void handle(OVERLAPPED *overlapped);

		Device &device;
		WINUSB_SETUP_PACKET setup;
		OVERLAPPED overlapped;
	};


	class BulkEndpoint;

	/**
		Buffer for transferring data to/from an endpoint
	*/
	class BulkBuffer : public LinkedListNode, public BufferImpl {
		friend class UsbHost_WinUSB::Device;
	public:
		BulkBuffer(BulkEndpoint &endpoint, int size);
		~BulkBuffer() override;

		bool startInternal(int size, Op op) override;
		void cancel() override;

	protected:
		void handle(OVERLAPPED *overlapped);

		BulkEndpoint &endpoint;

		OVERLAPPED overlapped[2];
		int index;
	};

	/**
		Bulk endpoint
	*/
	class BulkEndpoint : public LinkedListNode, public coco::Device {
		friend class UsbHost_WinUSB::Device;
		friend class BulkBuffer;
	public:
		BulkEndpoint(UsbHost_WinUSB::Device &device, int inAddress, int outAddress);
		BulkEndpoint(UsbHost_WinUSB::Device &device, int address) : BulkEndpoint(device,  usb::IN | address, usb::OUT | address) {}
		~BulkEndpoint();

		State state() override;
		Awaitable<State> untilState(State state) override;
		int getBufferCount() override;
		BulkBuffer &getBuffer(int index) override;

	protected:
		UsbHost_WinUSB::Device &device;
		int inAddress;
		int outAddress;

		// list of buffers
		LinkedList<BulkBuffer> buffers;
	};

	/**
		USB device as seen by the host. Connects itself to an actual USB device when it is plugged in and the
		filter on the device descriptor returns true.
	*/
	class Device : public UsbHostDevice, public Loop_Win32::CompletionHandler, public LinkedListNode {
		friend class UsbHost_WinUSB;
	public:

		/**
			Constructor
			@param host usb host
			@param filter filter to indicate if an usb device is handled by this device instance
		*/
		Device(UsbHost_WinUSB &host, std::function<bool (const usb::DeviceDescriptor &)> filter);

		~Device() override;

		State state() override;
		bool ready() {return this->stat == State::READY;}
		[[nodiscard]] Awaitable<State> untilState(State state) override;

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
		State stat = State::DISABLED;
		TaskList<State> stateTasks;

		// buffers for control endpoint
		LinkedList<ControlBuffer> controlBuffers;

		// bulk endpoints
		LinkedList<BulkEndpoint> bulkEndpoints;
	};

protected:
	Loop_Win32 &loop;
	std::map<std::string, Device *> deviceInfos;
	LinkedList<Device> devices;
};

} // namespace coco

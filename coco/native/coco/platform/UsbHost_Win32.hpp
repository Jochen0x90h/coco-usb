#pragma once

#include <coco/Stream.hpp>
#include <coco/usb.hpp>
#include <coco/UsbHostDevice.hpp>
#include <coco/platform/Loop_Win32.hpp>
#include <string>
#include <map>
#include <functional>


namespace coco {

/**
 * USB host implementation using win32 and WinUsb
 * https://learn.microsoft.com/en-us/windows/win32/api/winusb/ 
 */
class UsbHost_Win32 : public TimeHandler {
public:
	UsbHost_Win32(Loop_Win32 &loop);
	~UsbHost_Win32() override;

	void handle() override;


	class Device;

	class BulkEndpoint : public Stream, public LinkedListNode<BulkEndpoint> {
		friend class Device;
	public:
		BulkEndpoint(Device &device, int inAddress, int outAddress, bool packet = true);
		BulkEndpoint(Device &device, int address, bool packet = true) : BulkEndpoint(device, usb::IN | address, usb::OUT | address, packet) {}

		~BulkEndpoint() override;

		Awaitable<ReadParameters> read(void *data, int &size/*, bool packet*/) override;
		Awaitable<WriteParameters> write(void const *data, int size/*, bool packet*/) override;
		using Stream::read;
		using Stream::write;

	//protected:
		void startRead(void *data, int size);//, bool packet);
		void startWrite(const void *data, int size);//, bool packet);

		void handle(OVERLAPPED *overlapped);


		Device &device;
		int inAddress;
		int outAddress;

		bool reading = false;
		uint8_t *readData;
		int readSize;
		bool readPacket;
		uint8_t readBuffer[4096];
		OVERLAPPED readOverlapped;
		Waitlist<ReadParameters> readWaitlist;

		bool writing = false;
		const uint8_t *writeData;
		int writeSize;
		bool writePacket;
		uint8_t writeBuffer[4096];
		OVERLAPPED writeOverlapped;
		Waitlist<WriteParameters> writeWaitlist;
	};


	class Device : public UsbHostDevice, public CompletionHandler, public LinkedListNode<Device> {
	public:

		/**
		 * Constructor
		 * @param host usb host
		 * @param filter filter to indicate if an usb device is handled by this device instance
		 */
		Device(UsbHost_Win32 &host, std::function<bool (const usb::DeviceDescriptor &)> filter);

		~Device() override;

		State getState() override;
		bool isConnected() {return this->state == State::CONNECTED;}
		[[nodiscard]] Awaitable<State> targetState(State state) override;

		[[nodiscard]] Awaitable<ControlParameters> controlTransfer(usb::Setup const &setup,
			void *data, int size) override;

		//void getDescriptor(usb::DescriptorType type, void *data, int &size) override;


	//protected:
		void startTransfer(const usb::Setup &setup, void *data, int size);
		void handle(OVERLAPPED *overlapped) override;
		void disconnect();


		UsbHost_Win32 &host;
		std::function<bool (const usb::DeviceDescriptor &)> filter;

		// iterator of device list and flag for remove detection
		std::map<std::string, Device *>::iterator it;
		bool flag;
		
		// file handle and WinUsb interface
		HANDLE file = INVALID_HANDLE_VALUE;
		void *interface = nullptr;

		// state
		State state = State::DISCONNECTED;
		Waitlist<State> stateWaitlist;

		// control transfers
		bool transferring = false;
		uint8_t controlBuffer[512]; // size: https://www.techdesignforums.com/practice/technique/usb-3-0-protocol-layer-2/	
		OVERLAPPED controlOverlapped;
		Waitlist<ControlParameters> controlWaitlist;

		// bulk endpoints
		LinkedList<BulkEndpoint> bulkEndpoints;
	};

//protected:
	Loop_Win32 &loop;
	std::map<std::string, Device *> deviceInfos;
	LinkedList<Device> devices;
};

} // namespace coco

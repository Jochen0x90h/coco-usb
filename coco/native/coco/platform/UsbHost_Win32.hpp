#include <coco/Stream.hpp>
#include <coco/usb.hpp>
#include <coco/UsbHostDevice.hpp>
#include <coco/platform/Handler.hpp>
#include <string>
#include <map>
#include <functional>
#define NOMINMAX
#include <Windows.h>
#undef IN
#undef OUT
#undef interface


namespace coco {

/**
 * USB host implementation using win32 and WinUsb
 * https://learn.microsoft.com/en-us/windows/win32/api/winusb/ 
 */
class UsbHost_Win32 : public TimeHandler {
public:

	UsbHost_Win32();
	~UsbHost_Win32() override;

	void activate() override;


	class Device;

	class BulkEndpoint : public Stream, public YieldHandler, public LinkedListNode<BulkEndpoint> {
	public:

		BulkEndpoint(Device &device, int inAddress, int outAddress);
		BulkEndpoint(Device &device, int address) : BulkEndpoint(device, usb::IN | address, usb::OUT | address) {}

		~BulkEndpoint() override;

		Awaitable<ReadParameters> read(void *data, int &size) override;
		Awaitable<WriteParameters> write(void const *data, int size) override;
		using Stream::read;
		using Stream::write;

		void activate() override;

	//protected:

		Device &device;
		int inAddress;
		int outAddress;

		bool reading = false;
		OVERLAPPED readOverlapped;
		Waitlist<ReadParameters> readWaitlist;

		bool writing = false;
		OVERLAPPED writeOverlapped;
		Waitlist<WriteParameters> writeWaitlist;
	};


	class Device : public UsbHostDevice, public YieldHandler, public LinkedListNode<Device> {
	public:

		Device(UsbHost_Win32 &host, std::function<bool (const usb::DeviceDescriptor &)> filter);

		~Device() override;

		State getState() override;
		bool isConnected() {return this->state == State::CONNECTED;}
		[[nodiscard]] Awaitable<State> targetState(State state) override;

		[[nodiscard]] Awaitable<ControlParameters> controlTransfer(usb::RequestType requestType, uint8_t request,
			uint16_t value, uint16_t index, void *data, uint16_t length) override;

		//void getDescriptor(usb::DescriptorType type, void *data, int &size) override;

		void activate() override;

	//protected:
		void disconnect();

		UsbHost_Win32 &host;
		std::function<bool (const usb::DeviceDescriptor &)> filter;

		std::map<std::string, Device *>::iterator it;
		bool flag;
		HANDLE file = INVALID_HANDLE_VALUE;
		void *interface = nullptr;

		State state = State::DISCONNECTED;
		Waitlist<State> stateWaitlist;

		bool transferring = false;
		OVERLAPPED controlOverlapped;
		Waitlist<ControlParameters> controlWaitlist;

		LinkedList<BulkEndpoint> bulkEndpoints;
	};



	std::map<std::string, Device *> deviceInfos;
	LinkedList<Device> devices;
};

} // namespace coco

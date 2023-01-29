#pragma once

#include <coco/UsbDevice.hpp>
#include <coco/Stream.hpp>
#include <coco/platform/Loop_native.hpp>


namespace coco {

/**
 * Implementation of an USB device that simply writes info about the transfer operations to std::cout
 */
class UsbDevice_cout : public UsbDevice, public YieldHandler {
public:
	// number of endpoints without endpoint 0
	//static constexpr int ENDPOINT_COUNT = 7;

	/**
	 * Constructor
	 * @param getDescriptor callback for obtaining descriptors
	 * @param onSetConfiguration callback for setting the configuration (libusb_set_configuration() on host), always called from event loop
	 * @param onRequest callback for vendor specific request
	 */
	UsbDevice_cout(Loop_native &loop);/*
		std::function<ConstData (usb::DescriptorType)> const &getDescriptor,
		std::function<void (UsbDevice &usb, uint8_t bConfigurationValue)> const &onSetConfiguration,
		std::function<bool (uint8_t bRequest, uint16_t wValue, uint16_t wIndex)> const &onRequest);*/

	//void enableEndpoints(int inFlags, int outFlags) override;
	//[[nodiscard]] Awaitable<ReceiveParameters> receive(int index, void *data, int &size) override;
	//[[nodiscard]] Awaitable<SendParameters> send(int index, void const *data, int size) override;
	State getState() override;
	bool isConnected() {return this->state == State::CONNECTED;}
	[[nodiscard]] Awaitable<State> targetState(State state) override;

	[[nodiscard]] Awaitable<usb::Setup *> request(usb::Setup &setup) override;
	[[nodiscard]] Awaitable<ControlParameters> controlTransfer(void *data, int size) override;

	void acknowledge() override;
	void stall() override;


	class BulkEndpoint : public Stream, public LinkedListNode<BulkEndpoint> {
		friend class UsbDevice_cout;
	public:
	
		BulkEndpoint(UsbDevice_cout &device, int index) : device(device), index(index) {}

		~BulkEndpoint() override;

		Awaitable<ReadParameters> read(void *data, int &size) override;
		Awaitable<WriteParameters> write(void const *data, int size) override;
		using Stream::read;
		using Stream::write;

	protected:

		UsbDevice_cout &device;
		int index;
	};

protected:

	void handle() override;


	Loop_native &loop;

	// state
	State state = State::DISCONNECTED;
	Waitlist<State> stateWaitlist;

	// coroutines waiting for a control request
	Waitlist<usb::Setup *> requestWaitlist;

	bool readDescriptor = true;
	
	// binary/text extracted from the device descriptor
	bool text;

/*	
	std::function<void (UsbDevice &, uint8_t)> onSetConfiguration;

	// endpoints 1 - 7
	struct Endpoint {
		Waitlist<ReceiveParameters> receiveWaitlist;
		Waitlist<SendParameters> sendWaitlist;
	};

	Endpoint endpoints[ENDPOINT_COUNT];*/
	Waitlist<Stream::ReadParameters> readWaitlist;
	Waitlist<Stream::WriteParameters> writeWaitlist;
};

} // namespace coco

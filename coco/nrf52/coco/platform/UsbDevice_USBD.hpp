#pragma once

#include <coco/Stream.hpp>
#include <coco/UsbDevice.hpp>
#include <coco/platform/platform.hpp>
#include <coco/platform/Handler.hpp>
#include <coco/platform/Loop_RTC0.hpp>


namespace coco {

/**
 * Implementation of USB device for nRF52 using USBD peripheral
 * https://infocenter.nordicsemi.com/index.jsp?topic=%2Fps_nrf52840%2Fusbd.html&cp=4_0_0_5_34
 *
 * Resources:
 *	NRF_USBD
 */
class UsbDevice_USBD : public UsbDevice, public Handler {
public:
	UsbDevice_USBD(Loop_RTC0 &loop);
	~UsbDevice_USBD() override;

	State getState() override;
	bool isConnected() {return this->state == State::CONNECTED;}
	[[nodiscard]] Awaitable<State> targetState(State state) override;

	[[nodiscard]] Awaitable<usb::Setup *> request(usb::Setup &setup) override;
	[[nodiscard]] Awaitable<ControlParameters> controlTransfer(void *data, int size) override;

	void acknowledge() override;
	void stall() override;

	
	class BulkEndpoint : public Stream, public LinkedListNode<BulkEndpoint> {
		friend class UsbDevice_USBD;
	public:
		static constexpr int BUFFER_SIZE = 64;

		BulkEndpoint(UsbDevice_USBD &device, int inIndex, int outIndex, bool packet = true);
		BulkEndpoint(UsbDevice_USBD &device, int index, bool packet = true) : BulkEndpoint(device, index, index, packet) {}
		
		~BulkEndpoint() override;

		Awaitable<ReadParameters> read(void *data, int &size/*, bool packet*/) override;
		Awaitable<WriteParameters> write(void const *data, int size/*, bool packet*/) override;
		using Stream::read;
		using Stream::write;

	protected:
		void startRead(void *data, int size);//, bool packet);
		void startWrite(const void *data, int size);//, bool packet);


		UsbDevice_USBD &device;
		int inIndex;
		int outIndex;

		bool reading = false;
		uint8_t *readData;
		int readSize;
		bool readPacket;
		Waitlist<ReadParameters> readWaitlist;

		bool writing = false;
		const uint8_t *writeData;
		int writeSize;
		bool writePacket;
		Waitlist<WriteParameters> writeWaitlist;
	};

protected:
	void startIn(void *data, int size);
	void startOut(void *data, int size);
	void handle() override;


	// state
	State state = State::DISCONNECTED;
	Waitlist<State> stateWaitlist;

	// buffer for control and bulk transfers (we wait for DMA to finish because of errata 199)
	uint8_t buffer[64] __attribute__((aligned(4)));

	// coroutines waiting for a control request
	Waitlist<usb::Setup *> requestWaitlist;

	// control transfers	
	bool controlIn;
	uint8_t *controlData;
	int controlSize;
	Waitlist<ControlParameters> controlWaitlist;

	// bulk endpoints
	LinkedList<BulkEndpoint> bulkEndpoints;
};

} // namespace coco

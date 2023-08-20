#include "UsbDevice_USBD.hpp"
#include <coco/debug.hpp>
#include <coco/platform/nvic.hpp>


namespace coco {

UsbDevice_USBD::UsbDevice_USBD(Loop_RTC0 &loop) {
	//NRF_POWER->INTENSET = N(POWER_INTENSET_USBDETECTED) | N(POWER_INTENSET_USBREMOVED);

	NRF_USBD->INTENSET = N(USBD_INTENSET_USBRESET, Set)
		| N(USBD_INTENSET_USBEVENT, Set)
		| N(USBD_INTENSET_EP0SETUP, Set)
		| N(USBD_INTENSET_EP0DATADONE, Set)
		| N(USBD_INTENSET_EPDATA, Set);

	NRF_USBD->ENABLE = N(USBD_ENABLE_ENABLE, Enabled);

	// add to list of handlers
	loop.handlers.add(*this);
}

UsbDevice_USBD::~UsbDevice_USBD() {
}

UsbDevice::State UsbDevice_USBD::state() {
	return this->stat;
}

Awaitable<> UsbDevice_USBD::stateChange(int waitFlags) {
	if ((waitFlags & (1 << int(this->stat))) == 0)
		return {};
	return {this->stateTasks};
}

Awaitable<usb::Setup *> UsbDevice_USBD::request(usb::Setup &setup) {
	return {this->requestTasks, &setup};
}

void UsbDevice_USBD::acknowledge() {
	NRF_USBD->TASKS_EP0STATUS = TRIGGER;
}

void UsbDevice_USBD::stall() {
	NRF_USBD->TASKS_EP0STALL = TRIGGER;
}

void UsbDevice_USBD::handle() {
	if (nvic::pending(USBD_IRQn)) {
		if (NRF_USBD->EVENTS_USBRESET) {
			// clear pending interrupt flags at peripheral
			NRF_USBD->EVENTS_USBRESET = 0;

			// change state
			if (this->stat != State::DISABLED) {
				this->stat = State::DISABLED;

				// resume all coroutines waiting for disabled state
				this->stateTasks.doAll();
			}
		}
		if (NRF_USBD->EVENTS_USBEVENT) {
			// clear pending interrupt flag at peripheral
			NRF_USBD->EVENTS_USBEVENT = 0;

			// check cause
			if (NRF_USBD->EVENTCAUSE & N(USBD_EVENTCAUSE_READY, Ready)) {
				// usb is ready
				NRF_USBD->EVENTCAUSE = N(USBD_EVENTCAUSE_READY, Ready);

				// enable pullup
				NRF_USBD->USBPULLUP = N(USBD_USBPULLUP_CONNECT, Enabled);

				// enable control buffers
				for (auto &buffer : this->controlBuffers) {
					buffer.setReady(0);
				}
			}
		}

		/*
			Control IN transfer (host reads, device writes)
				EVENTS_EP0SETUP: Setup packet has arrived (contains direction)
				{
					TASKS_STARTEPIN[0]: Start DMA transfer from RAM to buffer
					EVENTS_ENDEPIN[0]: DMA transfer has finished
					EVENTS_EP0DATADONE: Data has been sent from buffer to the host
				}
				TASKS_EP0STATUS: Enter status stage

			Control OUT transfer (host writes, device reads)
				EVENTS_EP0SETUP: Setup packet has arrived (contains direction)
				{
					TASKS_EP0RCVOUT: Receive data from host to buffer
					EVENTS_EP0DATADONE: Data has been received
					TASKS_STARTEPOUT[0]: Start DMA transfer from buffer to RAM
					EVENTS_ENDEPOUT[0]: DMA transfer has finished
				}
		*/
		if (NRF_USBD->EVENTS_EP0SETUP) {
			// clear pending interrupt flag at peripheral
			NRF_USBD->EVENTS_EP0SETUP = 0;

			// setup request
			auto requestType = usb::RequestType(NRF_USBD->BMREQUESTTYPE);
			uint8_t request = NRF_USBD->BREQUEST;
			if (requestType == usb::RequestType::STANDARD_DEVICE_OUT && request == usb::Request::SET_ADDRESS) {
				// set address, handled by hardware
			} else if (requestType == usb::RequestType::STANDARD_DEVICE_OUT && request == usb::Request::SET_CONFIGURATION) {
				// set configuration
				//uint8_t configurationValue = NRF_USBD->WVALUEL;

				//  enable endpoints
				int inFlags = 1;
				int outFlags = 1;
				for (auto &endpoint : this->bulkEndpoints) {
					if (endpoint.inIndex > 0)
						inFlags |= 1 << endpoint.inIndex;
					if (endpoint.outIndex > 0)
						outFlags |= 1 << endpoint.outIndex;
				}
				NRF_USBD->EPINEN = inFlags;
				NRF_USBD->EPOUTEN = outFlags;

				// acknowledge the control transfer
				NRF_USBD->TASKS_EP0STATUS = TRIGGER;

				// set state
				this->stat = State::READY;

				// reset toggles, start OUT and enable buffers
				for (auto &endpoint : this->bulkEndpoints) {
					NRF_USBD->DTOGGLE = endpoint.inIndex | N(USBD_DTOGGLE_IO, In) | N(USBD_DTOGGLE_VALUE, Data0);
					NRF_USBD->DTOGGLE = endpoint.outIndex | N(USBD_DTOGGLE_IO, Out) | N(USBD_DTOGGLE_VALUE, Data0);

					// write any value to start receiving OUT transfers into intermediate buffer
					NRF_USBD->SIZE.EPOUT[endpoint.outIndex] = 0;

					// enable bulk buffers
					for (auto &buffer : endpoint.buffers) {
						buffer.setReady(0);
					}
				}

				// resume all coroutines waiting for ready state
				this->stateTasks.doAll();
			} else if (requestType == usb::RequestType::STANDARD_INTERFACE_OUT && request == usb::Request::SET_INTERFACE) {
				// set interface (interface index is in setup.index, alternate setting is in setup.value)

				// acknowledge
				NRF_USBD->TASKS_EP0STATUS = TRIGGER;
			} else {
				// resume first coroutine waiting for a control request
				this->requestTasks.doFirst([requestType, request](usb::Setup *setup) {
					uint16_t value = (NRF_USBD->WVALUEH << 8) | NRF_USBD->WVALUEL;
					uint16_t index = (NRF_USBD->WINDEXH << 8) | NRF_USBD->WINDEXL;
					uint16_t length = (NRF_USBD->WLENGTHH << 8) | NRF_USBD->WLENGTHL;
					*setup = {requestType, request, value, index, length};
					return true;
				});
			}
		}

		if (NRF_USBD->EVENTS_EP0DATADONE) {
			// clear pending interrupt flag at peripheral
			NRF_USBD->EVENTS_EP0DATADONE = 0;

			//if (this->controlIn) {
			if (this->inBusy & 1) {
				// control IN transfer (host reads)
				auto &list = this->controlTransfers;
				if (!list.empty()) {
					auto &buffer = *list.begin();
					if (buffer.writing) {
						// write/IN operation has finished
						buffer.finishWrite();
					}
				}

				// check if we need to start a new transfer
				if (!list.empty()) {
					auto &buffer = *list.begin();
					if (!buffer.writing)
						buffer.startWrite();
				}
			} else {
				// control OUT transfer (host writes)
				auto &list = this->controlTransfers;
				if (!list.empty()) {
					auto &buffer = *list.begin();
					buffer.finishRead();
				}
			}
		}

		/*
			Bulk IN transfer (host reads, device writes)
				TASKS_STARTEPIN[i]: Start DMA transfer from RAM to buffer
				EVENTS_ENDEPIN[i]: DMA transfer has finished
				EVENTS_EPDATA: Data has been sent from buffer to the host

			Bulk OUT transfer (host writes, device reads)
				EVENTS_EPDATA: Data has been received from the host into buffer
				TASKS_STARTEPOUT[i]: Start DMA transfer from buffer to RAM
				EVENTS_ENDEPOUT[i]: DMA transfer has finished
		*/
		if (NRF_USBD->EVENTS_EPDATA) {
			// clear pending interrupt flag at peripheral
			NRF_USBD->EVENTS_EPDATA = 0;

			// get and clear endpoint flags
			uint32_t dataStatus = NRF_USBD->EPDATASTATUS;
			NRF_USBD->EPDATASTATUS = dataStatus;

			// iterate over bulk IN buffers
			for (int i = 1; i < 8; ++i) {
/* todo: somehow this does not work, probably due to some hardware errata
				int flag = 0x1 << i;
				if (dataStatus & flag) {
					auto &transfers = this->inTransfers[i - 1];
					auto it = transfers.begin();
					if (it != transfers.end()) {//} && current.writing) {
						auto &current = *it;
						if (current.finishWrite()) {
							// write/IN operation has finished

							// check if more transfers are pending
							auto next = it;
							++next;
							if (next != transfers.end())
								next->startWrite();
							else
								this->inBusy &= ~flag;

							// set current buffer to ready state and resume waiting coroutines
							current.remove2();
							current.setReady();
						}
					}
				}
*/
				auto &list = this->inTransfers[i - 1];
				if (!list.empty()) {
					auto &buffer = *list.begin();
					int flag = 0x1 << buffer.endpoint.inIndex;
					if (buffer.writing && (dataStatus & flag) != 0) {
						// write/IN operation has finished
						buffer.finishWrite();
					}
				}

				// check if we need to start a new transfer
				if (!list.empty()) {
					auto &buffer = *list.begin();
					if (!buffer.writing) {
						buffer.startWrite();
					}
				}
			}

			// iterate over bulk OUT buffers
			for (int i = 1; i < 8; ++i) {
				int flag = 0x10000 << i;
				if (dataStatus & flag) {
					auto &transfers = this->outTransfers[i - 1];
					auto it = transfers.begin();
					if (it != transfers.end()) {
						// read/OUT operation has finished
						it->finishRead();
					} else {
						// store flag for pending OUT packet
						this->outAvailable |= flag;
					}
				}
			}
		}

		// clear pending interrupt flag at NVIC
		nvic::clear(USBD_IRQn);
	}
}


// ControlBufferBase

UsbDevice_USBD::ControlBufferBase::ControlBufferBase(uint8_t *data, int size, UsbDevice_USBD &device)
	: BufferImpl(data, size, device.stat)
	, device(device)
{
	device.controlBuffers.add(*this);
}

UsbDevice_USBD::ControlBufferBase::~ControlBufferBase() {
}

bool UsbDevice_USBD::ControlBufferBase::startInternal(int size, Op op) {
	if (this->stat != State::READY) {
		assert(this->stat != State::BUSY);
		return false;
	}

	// check if READ or WRITE flag is set
	assert((op & Op::READ_WRITE) != 0);

	this->partial = (op & Op::PARTIAL) != 0;

	this->transferIt = this->dat;
	this->transferEnd = this->dat + size;

	// start the transfer
	if ((op & Op::WRITE) == 0) {
		// read/OUT
		this->device.controlTransfers.add(*this);

		// control OUT transfer (host writes)
		NRF_USBD->TASKS_EP0RCVOUT = TRIGGER; // -> EP0DATADONE

		// now wait for data arriving from host
	} else {
		// write/IN
		this->device.controlTransfers.add(*this);

		// check if we can write immediately
		if ((this->device.inBusy & (0x1 << 0)) == 0)
			startWrite();
	}

	// set state
	setBusy();

	return true;
}

void UsbDevice_USBD::ControlBufferBase::cancel() {
	if (this->stat != State::BUSY)
		return;

	//setCancelled();
	this->remove2();

	// cancel takes effect immediately
	setReady(0);
}

void UsbDevice_USBD::ControlBufferBase::finishRead() {
	int i = 0;
	auto transferIt = this->transferIt;
	auto transferEnd = this->transferEnd;

	int received = NRF_USBD->SIZE.EPOUT[i];
	int toCopy = std::min(received, int(transferEnd - transferIt));
	NRF_USBD->EPOUT[i].PTR = intptr_t(transferIt);
	NRF_USBD->EPOUT[i].MAXCNT = toCopy;

	// start DMA transfer from RAM to buffer
	*(volatile uint32_t *)0x40027C1C = 0x00000082;
	NRF_USBD->TASKS_STARTEPOUT[i] = TRIGGER; // -> ENDEPOUT[i]

	// wait for end of transfer (easy solution and because of errata 199)
	while (!NRF_USBD->EVENTS_ENDEPOUT[i]);
	NRF_USBD->EVENTS_ENDEPOUT[i] = 0;
	*(volatile uint32_t *)0x40027C1C = 0x00000000;

	transferIt += toCopy; // NRF_USBD->EPOUT[i].AMOUNT

	if (transferIt < transferEnd && received >= BUFFER_SIZE) {
		// more to read
		this->transferIt = transferIt;

		// control OUT transfer (host writes)
		NRF_USBD->TASKS_EP0RCVOUT = TRIGGER; // -> EP0DATADONE
	} else {
		// read finished
		remove2();
		int transferred = transferIt - this->dat;

		// control endpoint: enter status stage
		NRF_USBD->TASKS_EP0STATUS = TRIGGER;

		setReady(transferred);
	}
}

void UsbDevice_USBD::ControlBufferBase::startWrite() {
	int i = 0;
	int toWrite = std::min(int(this->transferEnd - this->transferIt), BUFFER_SIZE);
	NRF_USBD->EPIN[i].PTR = intptr_t(this->transferIt);
	NRF_USBD->EPIN[i].MAXCNT = toWrite;

	// start DMA transfer from RAM to buffer
	*(volatile uint32_t *)0x40027C1C = 0x00000082;
	NRF_USBD->TASKS_STARTEPIN[i] = TRIGGER; // -> ENDEPIN[i]

	// wait for end of transfer (easy solution and because of errata 199)
	while (!NRF_USBD->EVENTS_ENDEPIN[i]);
	NRF_USBD->EVENTS_ENDEPIN[i] = 0;
	*(volatile uint32_t *)0x40027C1C = 0x00000000;

	this->writing = true;
	this->device.inBusy |= 0x1 << i;

	// now wait for EVENTS_EP0DATADONE
}

void UsbDevice_USBD::ControlBufferBase::finishWrite() {
	int i = 0;
	int transferred = NRF_USBD->EPIN[i].AMOUNT;
	auto transferIt = this->transferIt + transferred;
	auto transferEnd = this->transferEnd;

	// check if more to write or a zero packet is needed
	if (transferIt < transferEnd || (!this->partial && transferred == BUFFER_SIZE)) {
		// more to write or write zero packet
		this->transferIt = transferIt;
		this->startWrite();
		return;
	}

	// write finished
	remove2();

	// control endpoint: enter status stage
	if (!this->partial)
		NRF_USBD->TASKS_EP0STATUS = TRIGGER;

	setReady(transferIt - this->dat);

	this->writing = false;
	this->device.inBusy &= ~(0x1 << i);
}


// BulkBufferBase

UsbDevice_USBD::BulkBufferBase::BulkBufferBase(uint8_t *data, int size, BulkEndpoint &endpoint)
	: BufferImpl(data, size, endpoint.device.stat)
	, endpoint(endpoint)
{
	endpoint.buffers.add(*this);
}

UsbDevice_USBD::BulkBufferBase::~BulkBufferBase() {
}

bool UsbDevice_USBD::BulkBufferBase::startInternal(int size, Op op) {
	if (this->stat != State::READY) {
		assert(this->stat != State::BUSY);
		return false;
	}

	// check if READ or WRITE flag is set
	assert((op & Op::READ_WRITE) != 0);

	this->partial = (op & Op::PARTIAL) != 0;

	this->transferIt = this->dat;
	this->transferEnd = this->dat + size;

	// start the transfer
	auto &device = this->endpoint.device;
	if ((op & Op::WRITE) == 0) {
		// read/OUT
		int i = this->endpoint.outIndex;
		device.outTransfers[i - 1].add(*this);

		// check if a packet is already available
		int flag = 0x10000 << i;
		if (device.outAvailable & flag) {
			device.outAvailable &= ~flag;
			finishRead();
		}

		// now wait for data arriving from host
	} else {
		// write/IN
		int i = this->endpoint.inIndex;
		device.inTransfers[i - 1].add(*this);

		// check if we can write immediately
		int flag = 0x1 << i;
		if ((device.inBusy & flag) == 0) {
			startWrite();
			//device.inBusy |= flag;
		}
	}

	// set state
	setBusy();

	return true;
}

void UsbDevice_USBD::BulkBufferBase::cancel() {
	if (this->stat != State::BUSY)
		return;

	//setCancelled();
	this->remove2();

	// cancel takes effect immediately
	setReady(0);
}

void UsbDevice_USBD::BulkBufferBase::finishRead() {
	int i = this->endpoint.outIndex;
	auto transferIt = this->transferIt;
	auto transferEnd = this->transferEnd;

	int received = NRF_USBD->SIZE.EPOUT[i];
	int toCopy = std::min(received, int(transferEnd - transferIt));
	NRF_USBD->EPOUT[i].PTR = intptr_t(transferIt);
	NRF_USBD->EPOUT[i].MAXCNT = toCopy;

	// start DMA transfer from RAM to buffer
	*(volatile uint32_t *)0x40027C1C = 0x00000082;
	NRF_USBD->TASKS_STARTEPOUT[i] = TRIGGER; // -> ENDEPOUT[i]

	// wait for end of transfer (easy solution and because of errata 199)
	while (!NRF_USBD->EVENTS_ENDEPOUT[i]);
	NRF_USBD->EVENTS_ENDEPOUT[i] = 0;
	*(volatile uint32_t *)0x40027C1C = 0x00000000;

	transferIt += toCopy; // NRF_USBD->EPOUT[i].AMOUNT

	if (transferIt < transferEnd && received >= BUFFER_SIZE) {
		// more to read
		this->transferIt = transferIt;
	} else {
		// read finished
		remove2();
		int transferred = transferIt - this->dat;
		setReady(transferred);
	}
}

void UsbDevice_USBD::BulkBufferBase::startWrite() {
	int i = this->endpoint.inIndex;
	int toWrite = std::min(int(this->transferEnd - this->transferIt), BUFFER_SIZE);
	NRF_USBD->EPIN[i].PTR = intptr_t(this->transferIt);
	NRF_USBD->EPIN[i].MAXCNT = toWrite;

	// start DMA transfer from RAM to buffer
	*(volatile uint32_t *)0x40027C1C = 0x00000082;
	NRF_USBD->TASKS_STARTEPIN[i] = TRIGGER; // -> ENDEPIN[i]

	// wait for end of transfer (easy solution and because of errata 199)
	while (!NRF_USBD->EVENTS_ENDEPIN[i]);
	NRF_USBD->EVENTS_ENDEPIN[i] = 0;
	*(volatile uint32_t *)0x40027C1C = 0x00000000;

	this->writing = true;
	this->endpoint.device.inBusy |= 0x1 << i;

	// now wait for EVENTS_EPDATA
}

bool UsbDevice_USBD::BulkBufferBase::finishWrite() {
	int i = this->endpoint.inIndex;
	int transferred = NRF_USBD->EPIN[i].AMOUNT;
	auto transferIt = this->transferIt + transferred;
	auto transferEnd = this->transferEnd;

	// check if more to write or a zero packet is needed
	if (transferIt < transferEnd || (!this->partial && transferred == BUFFER_SIZE)) {
		// more to write or write zero packet
		this->transferIt = transferIt;
		this->startWrite();
		return false;
	}

	// write finished
	remove2();
	setReady(transferIt - this->dat);

	this->writing = false;
	this->endpoint.device.inBusy &= ~(0x1 << i);

	return true;
}


// BulkEndpoint

UsbDevice_USBD::BulkEndpoint::BulkEndpoint(UsbDevice_USBD &device, int inIndex, int outIndex)
	: device(device), inIndex(inIndex), outIndex(outIndex)
{
	device.bulkEndpoints.add(*this);
}

UsbDevice_USBD::BulkEndpoint::~BulkEndpoint() {
}

BufferDevice::State UsbDevice_USBD::BulkEndpoint::state() {
	return this->device.stat;
}

Awaitable<> UsbDevice_USBD::BulkEndpoint::stateChange(int waitFlags) {
	if ((waitFlags & (1 << int(this->device.stat))) == 0)
		return {};
	return {this->device.stateTasks};
}

int UsbDevice_USBD::BulkEndpoint::getBufferCount() {
	return this->buffers.count();
}

UsbDevice_USBD::BulkBufferBase &UsbDevice_USBD::BulkEndpoint::getBuffer(int index) {
	return this->buffers.get(index);
}

} // namespace coco

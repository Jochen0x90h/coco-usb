#include "UsbDevice_USBD.hpp"
#include <coco/debug.hpp>


namespace coco {

UsbDevice_USBD::UsbDevice_USBD(Loop_RTC0 &loop) {
	//NRF_POWER->INTENSET = N(POWER_INTENSET_USBDETECTED) | N(POWER_INTENSET_USBREMOVED);
	
	NRF_USBD->INTENSET = N(USBD_INTENSET_USBRESET, Set)
		| N(USBD_INTENSET_USBEVENT, Set)
		| N(USBD_INTENSET_EP0SETUP, Set)
		| N(USBD_INTENSET_EP0DATADONE, Set)
		| N(USBD_INTENSET_EPDATA, Set)
		//| 0xef << USBD_INTENSET_ENDEPOUT1_Pos // OUT endpoint 1-7
		//| 0xef << USBD_INTENSET_ENDEPIN1_Pos // IN endpoint 1-7
		;

	NRF_USBD->ENABLE = N(USBD_ENABLE_ENABLE, Enabled);

	// Errata 199: USBD cannot receive tasks during DMA, https://infocenter.nordicsemi.com/topic/errata_nRF52840_Rev2/ERR/nRF52840/Rev2/latest/anomaly_840_199.html
	//*(volatile uint32_t *)0x40027C1C = 0x00000082;

	// add to list of handlers
	loop.handlers.add(*this);
}

UsbDevice_USBD::~UsbDevice_USBD() {
}

UsbDevice::State UsbDevice_USBD::getState() {
	return this->state;
}

Awaitable<UsbDevice::State> UsbDevice_USBD::targetState(State state) {
	if (this->state == state)
		return {};
	return {this->stateWaitlist, state};
}

Awaitable<usb::Setup *> UsbDevice_USBD::request(usb::Setup &setup) {
	return {this->requestWaitlist, &setup};
}

static void cancelControlTransfer(UsbDevice::ControlParameters &p) {
	//auto &device = *reinterpret_cast<UsbDevice_USBD2 *>(p.context);
	p.remove();
}

Awaitable<UsbDevice::ControlParameters> UsbDevice_USBD::controlTransfer(void *data, int size) {
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
	if (this->controlIn) {
		// control IN transfer (host reads)
		startIn(data, size);
		return {this->controlWaitlist, data, size, this, cancelControlTransfer};
	} else {
		// control OUT transfer (host writes)
		startOut(data, size);
		return {this->controlWaitlist, data, size, this, cancelControlTransfer};
	}
}

void UsbDevice_USBD::acknowledge() {
	NRF_USBD->TASKS_EP0STATUS = TRIGGER;
}

void UsbDevice_USBD::stall() {
	NRF_USBD->TASKS_EP0STALL = TRIGGER;
}

void UsbDevice_USBD::startIn(void *data, int size) {
	// control IN transfer (host reads, device writes)
	auto d = reinterpret_cast<uint8_t *>(data);
	int toWrite = std::min(size, 64);
	std::copy(d, d + toWrite, this->buffer);
	NRF_USBD->EPIN[0].PTR = intptr_t(this->buffer);
	NRF_USBD->EPIN[0].MAXCNT = toWrite;
	
	// start DMA transfer from RAM to buffer
	NRF_USBD->TASKS_STARTEPIN[0] = TRIGGER; // -> ENDEPIN[0]
	
	// wait for end of transfer
	while (!NRF_USBD->EVENTS_ENDEPIN[0]);
	NRF_USBD->EVENTS_ENDEPIN[0] = 0;

	this->controlData = d;
	this->controlSize = size;
}

void UsbDevice_USBD::startOut(void *data, int size) {
	// control OUT transfer (host writes)
	NRF_USBD->TASKS_EP0RCVOUT = TRIGGER; // -> EP0DATADONE

	this->controlData = reinterpret_cast<uint8_t *>(data);
	this->controlSize = size;
}

void UsbDevice_USBD::handle() {
	if (isInterruptPending(USBD_IRQn)) {
		if (NRF_USBD->EVENTS_USBRESET) {
			// clear pending interrupt flags at peripheral
			NRF_USBD->EVENTS_USBRESET = 0;

			// change state
			if (this->state != State::DISCONNECTED) {
				this->state = State::DISCONNECTED;
				
				// resume all coroutines waiting for disconnected state
				this->stateWaitlist.resumeAll([](State state) {
					return state == State::DISCONNECTED;
				});
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
			}
		}

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
				//uint8_t configurationvalue = NRF_USBD->WVALUEL;

				//  enable endpoints
				int inFlags = 1;
				int outFlags = 1;
				for (auto &endpoint : this->bulkEndpoints) {
					inFlags |= 1 << endpoint.inIndex;
					outFlags |= 1 << endpoint.outIndex;
				}			
				NRF_USBD->EPINEN = inFlags;
				NRF_USBD->EPOUTEN = outFlags;

				// reset toggles and start OUT
				for (auto &endpoint : this->bulkEndpoints) {
					NRF_USBD->DTOGGLE = endpoint.inIndex | N(USBD_DTOGGLE_IO, In) | N(USBD_DTOGGLE_VALUE, Data0);
					NRF_USBD->DTOGGLE = endpoint.outIndex | N(USBD_DTOGGLE_IO, Out) | N(USBD_DTOGGLE_VALUE, Data0);

					// write any value to start receiving OUT transfers into intermediate buffer
					NRF_USBD->SIZE.EPOUT[endpoint.outIndex] = 0;
				}

				// acknowledge
				NRF_USBD->TASKS_EP0STATUS = TRIGGER;

				// change state and resume all coroutines waiting for connected state
				this->state = State::CONNECTED;
				this->stateWaitlist.resumeAll([](State state) {
					return state == State::CONNECTED;
				});
			} else if (requestType == usb::RequestType::STANDARD_INTERFACE_OUT && request == usb::Request::SET_INTERFACE) {
				// set interface (interface index is in setup.index, alternate setting is in setup.value)

				// acknowledge
				NRF_USBD->TASKS_EP0STATUS = TRIGGER;
			} else {
				// store direction of control transfer
				this->controlIn = (requestType & usb::RequestType::DIRECTION_MASK) == usb::RequestType::IN;

				// resume first coroutine waiting for a control request
				this->requestWaitlist.resumeFirst([requestType, request](usb::Setup *setup) {
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

			if (this->controlIn) {
				// control IN transfer (host reads)
				int transferred = NRF_USBD->EPIN[0].AMOUNT;
				auto controlData = this->controlData + transferred;
				int controlSize = this->controlSize - transferred;
				if (controlSize > 0) {
					// more to send to host
					startIn(controlData, controlSize);
				} else {
					// finished: enter status stage
					NRF_USBD->TASKS_EP0STATUS = TRIGGER;

					// resume waiting coroutine
					this->controlWaitlist.resumeFirst([](ControlParameters &p) {
						return true;
					});
				}
			} else {
				// control OUT transfer (host writes)
				int transferred = NRF_USBD->SIZE.EPOUT[0];
				int toWrite = std::min(transferred, this->controlSize);
				NRF_USBD->EPIN[0].PTR = intptr_t(this->buffer);
				NRF_USBD->EPIN[0].MAXCNT = toWrite;

				// start DMA from buffer to RAM
				NRF_USBD->TASKS_STARTEPOUT[0] = TRIGGER; // -> ENDEPOUT[0]

				// wait for end of transfer
				while(!NRF_USBD->EVENTS_ENDEPOUT[0]);
				NRF_USBD->EVENTS_ENDEPOUT[0] = 0;

				transferred = NRF_USBD->EPOUT[0].AMOUNT;
				auto controlData = this->controlData;
				int controlSize = this->controlSize;
				int t = std::min(controlSize, int(transferred));
				std::copy(this->buffer, this->buffer + t, controlData);
				this->controlData += t;
				this->controlSize -= t;
				if (controlSize > 0) {
					// more to receive from  host
					startOut(controlData, controlSize);
				} else {
					// finished: enter status stage
					NRF_USBD->TASKS_EP0STATUS = TRIGGER;

					// resume waiting coroutine
					this->controlWaitlist.resumeFirst([](ControlParameters &p) {
						return true;
					});
				}
			}
		}

		if (NRF_USBD->EVENTS_EPDATA) {
			// clear pending interrupt flag at peripheral
			NRF_USBD->EVENTS_EPDATA = 0;
			uint32_t dataStatus = NRF_USBD->EPDATASTATUS;
			NRF_USBD->EPDATASTATUS = dataStatus;

			// iterate over bulk endpoints
			for (BulkEndpoint &endpoint : this->bulkEndpoints) {		
				if (dataStatus & (0x10000 << endpoint.outIndex)) {
					// read/OUT operation has finished
					int i = endpoint.outIndex;
					int transferred = NRF_USBD->SIZE.EPOUT[i];
					NRF_USBD->EPOUT[i].PTR = intptr_t(this->buffer);
					NRF_USBD->EPOUT[i].MAXCNT = transferred;
	
					// start DMA transfer from RAM to buffer
					NRF_USBD->TASKS_STARTEPOUT[i] = TRIGGER; // -> ENDEPOUT[0]
	
					// wait for end of transfer
					while (!NRF_USBD->EVENTS_ENDEPOUT[i]);
					NRF_USBD->EVENTS_ENDEPOUT[i] = 0;

					transferred = NRF_USBD->EPOUT[i].AMOUNT;
					auto readData = endpoint.readData;
					int readSize = endpoint.readSize;
					int t = std::min(readSize, int(transferred));
					//memcpy(readData, this->readBuffer, t);
					std::copy(this->buffer, this->buffer + t, readData);
					readData += t;
					readSize -= t;
					if ((endpoint.readPacket || readSize > 0) && transferred >= BulkEndpoint::BUFFER_SIZE) {
						// more to read
						endpoint.startRead(readData, readSize/*, endpoint.readPacket*/);
					} else {
						// resume waiting coroutine			
						endpoint.readWaitlist.resumeFirst([readData](Stream::ReadParameters &p) {
							*p.size = readData - reinterpret_cast<uint8_t *>(p.data);
							return true;
						});

						endpoint.reading = false;

						// start next read operation
						endpoint.readWaitlist.visitFirst([&endpoint](Stream::ReadParameters &p) {
							endpoint.startRead(p.data, *p.size/*, p.packet*/);
						});
					}
				}
				if (dataStatus & (0x1 << endpoint.inIndex)) {
					// write/IN operation has finished
					int transferred = NRF_USBD->EPIN[endpoint.inIndex].AMOUNT;
					auto writeData = endpoint.writeData + transferred;
					int writeSize = endpoint.writeSize - transferred;
					if (writeSize > 0 || (endpoint.writePacket && transferred > 0 && (transferred & (BulkEndpoint::BUFFER_SIZE - 1)) == 0)) { //todo: get packet size from endpoint descriptor
						// more to write
						endpoint.startWrite(writeData, writeSize/*, endpoint.writePacket*/);
					} else {
						// resume waiting coroutine			
						endpoint.writeWaitlist.resumeFirst([](Stream::WriteParameters &p) {
							return true;
						});		

						endpoint.writing = false;
					
						// start next write operation
						endpoint.writeWaitlist.visitFirst([&endpoint](Stream::WriteParameters &p) {
							endpoint.startWrite(p.data, p.size/*, p.packet*/);
						});						
					}
				}
			}
		}

		// clear pending interrupt flag at NVIC
		clearInterrupt(USBD_IRQn);
	}
}


// BulkEndpoint

UsbDevice_USBD::BulkEndpoint::BulkEndpoint(UsbDevice_USBD &device, int inIndex, int outIndex, bool packet)
	: device(device), inIndex(inIndex), outIndex(outIndex), readPacket(packet), writePacket(packet)
{
	device.bulkEndpoints.add(*this);
}

UsbDevice_USBD::BulkEndpoint::~BulkEndpoint() {	
}

static void cancelRead(Stream::ReadParameters &p) {
	//auto &endpoint = *reinterpret_cast<UsbDevice_USBD2::BulkEndpoint *>(p.context);

	p.remove();
}

Awaitable<Stream::ReadParameters> UsbDevice_USBD::BulkEndpoint::read(void *data, int &size/*, bool packet*/) {
	if (!this->device.isConnected())
		return {};
	if (!this->reading)
		startRead(data, size/*, packet*/);
	return {this->readWaitlist, data, &size, /*packet, */this, cancelRead};
}

static void cancelWrite(Stream::WriteParameters &p) {
	//auto &endpoint = *reinterpret_cast<UsbDevice_USBD2::BulkEndpoint *>(p.context);

	p.remove();
}

Awaitable<Stream::WriteParameters> UsbDevice_USBD::BulkEndpoint::write(void const *data, int size/*, bool packet*/) {
	if (!this->device.isConnected())
		return {};
	if (!this->writing)
		startWrite(data, size/*, packet*/);
	return {this->writeWaitlist, data, size, /*packet,*/ this, cancelWrite};
}

void UsbDevice_USBD::BulkEndpoint::startRead(void *data, int size/*, bool packet*/) {
	// bulk OUT transfer
	this->reading = true;
	this->readData = reinterpret_cast<uint8_t *>(data);
	this->readSize = size;
	//this->readPacket = packet;
}

void UsbDevice_USBD::BulkEndpoint::startWrite(const void *data, int size/*, bool packet*/) {
	// bulk IN transfer
	auto d = reinterpret_cast<const uint8_t *>(data);
	int toWrite = std::min(size, BUFFER_SIZE);
	std::copy(d, d + toWrite, this->device.buffer);
	int i = this->inIndex;
	NRF_USBD->EPIN[i].PTR = intptr_t(this->device.buffer);
	NRF_USBD->EPIN[i].MAXCNT = toWrite;
	
	// start DMA transfer from RAM to buffer
	NRF_USBD->TASKS_STARTEPIN[i] = TRIGGER; // -> ENDEPIN[0]
	
	// wait for end of transfer
	while (!NRF_USBD->EVENTS_ENDEPIN[i]);
	NRF_USBD->EVENTS_ENDEPIN[i] = 0;

	this->writing = true;
	this->writeData = d;
	this->writeSize = size;
	//this->writePacket = packet;
}

} // namespace coco

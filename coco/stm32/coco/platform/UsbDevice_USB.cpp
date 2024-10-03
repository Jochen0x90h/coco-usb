#include "UsbDevice_USB.hpp"
#include <coco/debug.hpp>
#include <coco/platform/nvic.hpp>


#ifdef USB
namespace coco {
namespace {

/*
	Glossary
		ZLP: Zero Length Packet

	Packet Memory
		see https://libopencm3.org/docs/latest/stm32l4/html/st__usbfs__v2_8h.html
		layout:
		table of buffer descriptors
		buffers for endpoint 0
		buffers for endpoint 1
		buffers for endpoint 2
		...
*/

struct BufferDescriptor {
	volatile uint16_t offset;
	volatile uint16_t size;

	int transferred() const {return this->size & 0x3ff;}
};

struct EndpointBufferDescriptors {
	BufferDescriptor tx; // offset, size
	BufferDescriptor rx; // offset, size, blocks (count and flag)
};
const auto ENDPOINT_BUFFER_DESCRIPTORS = (EndpointBufferDescriptors *)USB_PMAADDR;
constexpr int TABLE_SIZE = 8 * sizeof(EndpointBufferDescriptors);

const auto CONTROL_RX_BUFFER = (volatile uint16_t *)(USB_PMAADDR + TABLE_SIZE);

inline volatile uint16_t &getEndpointRegister(int ep) {return (&USB->EP0R)[ep * 2];}


// send data to the host
void usbSend(int ep, const void *data, int size) {
	auto &bd = ENDPOINT_BUFFER_DESCRIPTORS[ep].tx;

	// copy data from memory into tx buffer
	const uint16_t *src = (const uint16_t*)data;
	volatile uint16_t *dst = (volatile uint16_t *)(USB_PMAADDR + bd.offset);
	volatile uint16_t *end = dst + ((size + 1) >> 1);
	while (dst != end) {
		*dst = *src;
		++src;
		++dst;
	}

	// set size of packet in tx buffer descriptor
	bd.size = size;

	// indicate that we are ready to send
	// keep DTOG_RX DTOG_TX STAT_RX, cancel STAT_TX, clear CTR_TX, don't clear CTR_RX, toggle STAT_TX.VALID
	auto &EPxR = getEndpointRegister(ep);
	EPxR = ((EPxR & ~(USB_EP_DTOG_RX | USB_EP_DTOG_TX | USB_EPRX_STAT | USB_EP_CTR_TX)) | USB_EP_CTR_RX) ^ USB_EP_TX_VALID;
}

void usbAckSend(int ep) {
	// keep DTOG_RX DTOG_TX STAT_RX STAT_TX, clear CTR_TX, don't clear CTR_RX
	auto &EPxR = getEndpointRegister(ep);
	EPxR = ((EPxR & ~(USB_EP_DTOG_RX | USB_EP_DTOG_TX | USB_EPRX_STAT | USB_EPTX_STAT | USB_EP_CTR_TX)) | USB_EP_CTR_RX);
}

// indicate that we want to receive data from the host
void usbReceive(int ep) {
	// keep DTOG_RX DTOG_TX STAT_TX, cancel STAT_RX, clear CTR_RX, don't clear CTR_TX, toggle STAT_RX.STALL
	auto &EPxR = getEndpointRegister(ep);
	EPxR = ((EPxR & ~(USB_EP_DTOG_RX | USB_EP_DTOG_TX | USB_EPTX_STAT | USB_EP_CTR_RX)) | USB_EP_CTR_TX) ^ USB_EP_RX_VALID;
}

void usbAckReceive(int ep) {
	// keep DTOG_RX DTOG_TX STAT_RX STAT_TX, clear CTR_RX, don't clear CTR_TX
	auto &EPxR = getEndpointRegister(ep);
	EPxR = ((EPxR & ~(USB_EP_DTOG_RX | USB_EP_DTOG_TX | USB_EPRX_STAT | USB_EPTX_STAT | USB_EP_CTR_RX)) | USB_EP_CTR_TX);
}

void usbFinishReceive(int ep, void *data, int size) {
	auto &bd = ENDPOINT_BUFFER_DESCRIPTORS[ep].rx;

	// copy data from rx buffer into memory
	const volatile uint16_t *src = (const volatile uint16_t *)(USB_PMAADDR + bd.offset);
	uint16_t *dst = (uint16_t*)data;
	uint16_t *end = dst + ((size + 1) >> 1);
	while (dst != end) {
		*dst = *src;
		++src;
		++dst;
	}
}


// prepare status in transfer for control endpoint 0 (send a ZLP)
inline void usbControlStatusIn() {
	ENDPOINT_BUFFER_DESCRIPTORS[0].tx.size = 0;

	// keep DTOG_RX DTOG_TX STAT_RX, cancel STAT_TX, clear CTR_RX CTR_TX, toggle STAT_TX.VALID
	USB->EP0R = ((USB->EP0R & ~(USB_EP_DTOG_RX | USB_EP_DTOG_TX | USB_EPRX_STAT | USB_EP_CTR_RX | USB_EP_CTR_TX)) ) ^ USB_EP_TX_VALID;
}

// prepare status out transfer for control endpoint 0 (expect a ZLP)
inline void usbControlStatusOut() {
	// keep DTOG_RX DTOG_TX STAT_TX, cancel STAT_RX, clear CTR_RX CTR_TX, set EP_KIND (STATUS_OUT), toggle STAT_RX.VALID
	USB->EP0R = ((USB->EP0R & ~(USB_EP_DTOG_RX | USB_EP_DTOG_TX | USB_EPTX_STAT | USB_EP_CTR_RX | USB_EP_CTR_TX)) | USB_EP_KIND) ^ USB_EP_RX_VALID;
}

// stall both directions of control endpoint 0
inline void usbControlStall() {
	// keep DTOG_RX DTOG_TX, cancel STAT_RX STAT_TX, clear CTR_RX CTR_TX EP_KIND (STATUS_OUT), toggle STAT_RX.STALL STAT_TX.STALL
	USB->EP0R = ((USB->EP0R & ~(USB_EP_DTOG_RX | USB_EP_DTOG_TX | USB_EP_CTR_RX | USB_EP_CTR_TX | USB_EP_KIND)) ) ^ (USB_EP_RX_STALL | USB_EP_TX_STALL);
}


} // anonymous namespace


// USB_LP_IRQn: All USB events such as correct transfer, reset etc.
// USB_HP_IRQn: Correct transfer for isochronous and double buffered bulk transfers
// USBWakeUp_IRQn: USB Wakeup through EXTI line Interrupt

#ifdef STM32G4
#define USB_IRQn USB_LP_IRQn
#endif


// UsbDevice_USB

UsbDevice_USB::UsbDevice_USB(Loop_Queue &loop)
	: UsbDevice(State::OPENING), loop(loop)
{
#ifdef STM32F0
	// enable HSI48 (gets enabled automatically)

	// set HSI48 as USB clock source (is default)
	//RCC->CFGR3 = RCC->CFGR3 & ~RCC_CFGR3_USBSW_Msk;

	// enable clock of USB and CRS (clock recovery system)
	RCC->APB1ENR = RCC->APB1ENR | RCC_APB1ENR_CRSEN | RCC_APB1ENR_USBEN;
#endif
#ifdef STM32G4
	// enable HSI48 and wait until ready
	RCC->CRRCR = RCC_CRRCR_HSI48ON;
	while ((RCC->CRRCR & RCC_CRRCR_HSI48RDY) == 0);

	// set HSI48 as USB clock source (is default)
	//RCC->CCIPR = RCC->CCIPR & ~RCC_CCIPR_CLK48SEL_Msk;

	// enable clock of USB and CRS (clock recovery system)
	RCC->APB1ENR1 = RCC->APB1ENR1 | RCC_APB1ENR1_CRSEN | RCC_APB1ENR1_USBEN;
#endif

	// enable automatic trimming and oscillator clock for the frequency error counter
	CRS->CR = CRS->CR | CRS_CR_AUTOTRIMEN | CRS_CR_CEN;

	// switch on usb transceiver, but keep reset (reference manual: System and power-on reset)
	USB->CNTR = USB_CNTR_FRES;

	// wait for at least 1us (see data sheet: USB startup time)
	debug::sleep(1us);

	// enable pull-up resistor to start enumeration
	USB->BCDR = USB_BCDR_DPPU;

	// exit reset of usb
	USB->CNTR = 0;

	// clear interrupts
	USB->ISTR = 0;

	// set buffer descriptor table offset inside packet memory (which starts at USB_PMAADDR)
	USB->BTABLE = 0;

	// enable correct transfer and reset interrupts
	USB->CNTR = USB_CNTR_CTRM | USB_CNTR_RESETM;

	// enable USB interrupt
	nvic::setPriority(USB_IRQn, nvic::Priority::HIGH);
	nvic::enable(USB_IRQn);
}

UsbDevice_USB::~UsbDevice_USB() {
}

//StateTasks<const Device::State, Device::Events> &UsbDevice_USB::getStateTasks() {
//	return makeConst(this->st);
//}

/*Device::State UsbDevice_USB::state() {
	return this->stat;
}

Awaitable<Device::Condition> UsbDevice_USB::until(Condition condition) {
	// check if IN_* condition is met
	if ((int(condition) >> int(this->stat.load())) & 1)
		return {}; // don't wait
	return {this->stateTasks, condition};
}*/

usb::Setup UsbDevice_USB::getSetup() {
	int s0 = CONTROL_RX_BUFFER[0];
	auto requestType = usb::RequestType(s0);
	uint8_t request = s0 >> 8;
	uint16_t value = CONTROL_RX_BUFFER[1];
	uint16_t index = CONTROL_RX_BUFFER[2];
	uint16_t length = CONTROL_RX_BUFFER[3];
	return {requestType, request, value, index, length};
}

void UsbDevice_USB::acknowledge() {
	// acknowledge using ZLP of opposite direction
	if (this->controlMode == Mode::DATA_IN)
		usbControlStatusOut();
	else if (this->controlMode == Mode::DATA_OUT)
		usbControlStatusIn();
	else
		assert(false);
}

void UsbDevice_USB::stall() {
	usbControlStall();
}

void UsbDevice_USB::USB_IRQHandler() {
	int istr = USB->ISTR;
	if ((istr & USB_ISTR_RESET) != 0) {
		// clear interrupt flag
		USB->ISTR = ~USB_ISTR_RESET;
		//debug::setBlue();

		// setup buffers for endpoint 0 (tx count is set when actually sending data)
		auto &ebd = ENDPOINT_BUFFER_DESCRIPTORS[0];
		ebd.rx.offset = TABLE_SIZE;
		ebd.rx.size = 0x8000 | (1 << 10); // rx buffer size is 64
		ebd.tx.offset = TABLE_SIZE + 64;
		ebd.tx.size = 64;

		// setup control endpoint 0
		USB->EP0R = USB_EP_RX_VALID | USB_EP_TX_STALL | USB_EP_CONTROL | 0;

		// enable usb at usb address 0
		USB->DADDR = USB_DADDR_EF | 0;
	}

	int ep0r = USB->EP0R;
	if (ep0r & USB_EP_CTR_RX) {
		// received a packet from the host on endpoint 0
		usbAckReceive(0);

		if (ep0r & USB_EP_SETUP) {
			// received a setup request from host
			int s0 = CONTROL_RX_BUFFER[0];
			auto requestType = usb::RequestType(s0);
			uint8_t request = s0 >> 8;

			if (requestType == usb::RequestType::STANDARD_DEVICE_OUT && request == usb::Request::SET_ADDRESS) {
				// address is value of setup packet, keep in control receive buffer

				// enter status stage by sending ZLP of opposite direction
				usbControlStatusIn();
				this->controlMode = Mode::SET_ADDRESS;
			} else if (requestType == usb::RequestType::STANDARD_DEVICE_OUT && request == usb::Request::SET_CONFIGURATION) {
				// set configuration
				//uint8_t configurationValue = setup->value & 0xff;

				//  enable endpoints
				int offset = TABLE_SIZE + 2 * 64;
				for (auto &endpoint : this->endpoints) {
					if (endpoint.inIndex > 0) {
						auto &descriptor = ENDPOINT_BUFFER_DESCRIPTORS[endpoint.inIndex];
						descriptor.tx.offset = offset;
						descriptor.tx.size = 64;
						offset += 64;

						// configure tx (in) endpoint: stall send, clear other toggle bits
						auto &EPxR = getEndpointRegister(endpoint.inIndex);
						EPxR = ((EPxR
								& ~(USB_EP_CTR_TX // clear TX correct transfer flag
									| USB_EP_TYPE_MASK // clear endpoint type
									| USB_EP_KIND // clear endpoint kind
									| USB_EPADDR_FIELD // clear endpoint index (address)
									| USB_EP_DTOG_RX // keep RX toggle (0 = keep, 1 = toggle)
									| USB_EPRX_STAT)) // keep RX status (0 = keep, 1 = toggle)
								| int(endpoint.type)/*USB_EP_BULK*/ // set endpoint type
								| endpoint.inIndex) // set endpoint index (address)
							^ USB_EP_TX_NAK // set TX status to NAK (nothing to send)
							^ 0; // clear TX toggle (toggle when set)
					}
					if (endpoint.outIndex > 0) {
						auto &descriptor = ENDPOINT_BUFFER_DESCRIPTORS[endpoint.outIndex];
						descriptor.rx.offset = offset;
						descriptor.rx.size = 0x8000 | (1 << 10); // rx buffer size is 64
						offset += 64;

						// configure rx (out) endpoint: ready to receive, clear other toggle bits
						auto &EPxR = getEndpointRegister(endpoint.outIndex);
						EPxR = ((EPxR
								& ~(USB_EP_CTR_RX // clear RX correct transfer flag
									| USB_EP_TYPE_MASK // clear endpoint type
									| USB_EP_KIND // clear endpoint kind
									| USB_EPADDR_FIELD // clear endpoint index (address)
									| USB_EP_DTOG_TX // keep TX toggle (0 = keep, 1 = toggle)
									| USB_EPTX_STAT)) // keep TX status (0 = keep, 1 = toggle)
								| int(endpoint.type)/*USB_EP_BULK*/ // set endpoint type
								| endpoint.outIndex) // set endpoint index (address)
							^ USB_EP_RX_VALID // set RX status to VALID (ready to receive)
							^ 0; // clear RX toggle (toggle when set)
					}
				}

				// enter status stage by sending ZLP of opposite direction
				usbControlStatusIn();
				this->controlMode = Mode::STATUS;

				// start transfers that are already waiting
				for (int ep = 1; ep < 8; ++ep) {
					auto &transfers = this->transfers[ep - 1];
					auto inBuffer = transfers.in.frontOrNull();
					if (inBuffer != nullptr)
						inBuffer->startWrite();
					auto outBuffer = transfers.out.frontOrNull();
					if (outBuffer != nullptr)
						usbReceive(ep);
				}

				// set state
				this->stat = State::READY;

				// push this to the event handler queue so that the application gets notified about the sate change in UsbDevice_USB::handle()
				Events e = this->events;
				this->events = e | Events::ENTER_READY;
				if (e == Events::NONE)
					this->loop.push(*this);
			} else if (requestType == usb::RequestType::STANDARD_INTERFACE_OUT && request == usb::Request::SET_INTERFACE) {
				// set interface (interface index is in setup.index, alternate setting is in setup.value)

				// enter status stage by sending ZLP of opposite direction
				usbControlStatusIn();
			} else {
				this->controlMode = (requestType & usb::RequestType::DIRECTION_MASK) == usb::RequestType::OUT ? Mode::DATA_OUT : Mode::DATA_IN;

				// push this to the event handler queue so that the control request gets forwarded to the application in UsbDevice_USB::handle()
				Events e = this->events;
				this->events = e | Events::REQUEST;
				if (e == Events::NONE)
					this->loop.push(*this);
			}
		} else {
			// received data from host
			switch (this->controlMode) {
			case Mode::DATA_OUT:
				// control OUT transfer completed
				this->controlTransfers.pop(
					[](ControlBufferBase &buffer) {
						return buffer.finishRead();
					},
					[](ControlBufferBase &next) {
						// start next buffer, only allowed when previus buffer had PARTIAL flag
						assert(this->controlMode == Mode::DATA_OUT);
						usbReceive(0); // start read
					}
				);
				break;
			//case Mode::STATUS_IN:
			default:
				// status stage of IN control transfer completed
				//debug::setGreen();
				this->controlMode = Mode::IDLE;
				usbControlStall();
				break;
			}
		}
	}
	if (ep0r & USB_EP_CTR_TX) {
		// sent data to host
		usbAckSend(0);

		switch (this->controlMode) {
		case Mode::DATA_IN:
			// control IN transfer completed
			this->controlTransfers.pop(
				[](ControlBufferBase &buffer) {
					return buffer.finishWrite();
				},
				[](ControlBufferBase &next) {
					// start next buffer, only allowed when previus buffer had PARTIAL flag
					assert(this->controlMode == Mode::DATA_IN);
					next.startWrite();
				}
			);
			break;
		case Mode::SET_ADDRESS:
			// status stage of "set address" OUT control transfer complete, now set address
			USB->DADDR = USB_DADDR_EF | CONTROL_RX_BUFFER[1];
			// fall through
		default:
			// status stage of OUT control transfer complete
			this->controlMode = Mode::IDLE;
			usbControlStall();
			break;
		}
	}

	for (int ep = 1; ep < 8; ++ep) {
		int EPxR = getEndpointRegister(ep);
		if (EPxR & USB_EP_CTR_RX) {
			// OUT transfer completed (received from host)
			usbAckReceive(ep);
			auto &transfer = this->transfers[ep - 1];

			if (transfer.out.pop(
				[](BufferBase &buffer) {
					return buffer.finishRead();
				},
				[ep](BufferBase &next) {
					// start next buffer
					usbReceive(ep); // start read
				}
			) == -1) {
				// store flag that there is a pending OUT packet
				transfer.outAvailable = true;
			}
		}
		if (EPxR & USB_EP_CTR_TX) {
			// IN transfer completed (sent to host)
			usbAckSend(ep);
			auto &transfer = this->transfers[ep - 1];

			transfer.in.pop(
				[](BufferBase &buffer) {
					return buffer.finishWrite();
				},
				[](BufferBase &next) {
					// start next buffer
					next.startWrite();
				}
			);
		}
	}
}

// called from event loop to notify app about state changes and control requests
void UsbDevice_USB::handle() {
	State state = this->stat;
	Events events = this->events.exchange(Events::NONE);

	// set state and resume all coroutines waiting for state change
	if ((events & Events::ENTER_ANY) != 0) {
		for (auto &endpoint : this->endpoints)
			endpoint.st.set(state, events);
	}
	this->st.set(state, events);
}


// ControlBufferBase

UsbDevice_USB::ControlBufferBase::ControlBufferBase(uint8_t *data, int capacity, UsbDevice_USB &device)
	: Buffer(data, capacity, device.stat)
	, device(device)
{
	device.controlBuffers.add(*this);
}

UsbDevice_USB::ControlBufferBase::~ControlBufferBase() {
}

bool UsbDevice_USB::ControlBufferBase::start(Op op) {
	if (this->st.state != State::READY) {
		assert(this->p.state != State::BUSY);
		return false;
	}

	// check if READ or WRITE flag is set
	assert((op & Op::READ_WRITE) != 0);
	this->op = op;

	// start the transfer
	auto &device = this->device;
	this->transferIt = this->p.data;
	if ((op & Op::WRITE) == 0) {
		// read/OUT
		this->transferEnd = this->p.data + this->p.capacity;
		if (device.controlMode != UsbDevice_USB::Mode::DATA_OUT) {
			assert(false);
			return false;
		}

		// add to list of pending transfers and start immediately if list was empty
		if (device.controlTransfers.push(nvic::Guard(USB_IRQn), *this))
			usbReceive(0);

		// now wait for data arriving from host
	} else {
		// write/IN
		this->transferEnd = this->p.data + this->p.size;
		if (device.controlMode != UsbDevice_USB::Mode::DATA_IN) {
			assert(false);
			return false;
		}

		// add to list of pending transfers and start immediately if list was empty
		if (device.controlTransfers.push(nvic::Guard(USB_IRQn), *this))
			startWrite();
	}

	// set state
	setBusy();

	return true;
}

bool UsbDevice_USB::ControlBufferBase::cancel() {
	if (this->st.state != State::BUSY)
		return false;
	auto &device = this->device;

	// remove from pending transfers even if active
	device.controlTransfers.remove(nvic::Guard(USB_IRQn), *this);

	// cancel takes effect immediately
	setReady(0);
	return true;
}

bool UsbDevice_USB::ControlBufferBase::finishRead() {
	int ep = 0;
	auto transferIt = this->transferIt;
	auto transferEnd = this->transferEnd;

	int received = ENDPOINT_BUFFER_DESCRIPTORS[ep].rx.transferred();
	int toCopy = std::min(received, int(transferEnd - transferIt));
	usbFinishReceive(ep, transferIt, toCopy);
	transferIt += toCopy;

	if (transferIt < transferEnd && received >= BUFFER_SIZE) {
		// more to read
		this->transferIt = transferIt;

		// wait for next control OUT transfer (host writes)
		usbReceive(ep);

		// not finished yet
		return false;
	}

	// read operation has finished: set number of transferred bytes
	this->p.size = transferIt - this->p.data;

	if ((this->op & Op::PARTIAL) == 0) {
		// enter status stage by sending ZLP of opposite direction
		usbControlStatusIn();
		this->device.controlMode = UsbDevice_USB::Mode::STATUS;
	}

	// push finished buffer to event loop so that ControlBufferBase::handle() gets called from the event loop
	this->device.loop.push(*this);

	// finished
	return true;
}

void UsbDevice_USB::ControlBufferBase::startWrite() {
	int ep = 0;
	int toWrite = std::min(int(this->transferEnd - this->transferIt), BUFFER_SIZE);
	usbSend(ep, this->transferIt, toWrite);

	// now wait for USB_EP_CTR_TX
}

bool UsbDevice_USB::ControlBufferBase::finishWrite() {
	int ep = 0;
	int transferred = ENDPOINT_BUFFER_DESCRIPTORS[ep].tx.size;
	auto transferIt = this->transferIt + transferred;
	auto transferEnd = this->transferEnd;
	bool partial = (this->op & Op::PARTIAL) != 0;

	// check if more to write or a zero packet is needed
	if (transferIt < transferEnd || (!partial && transferred == BUFFER_SIZE)) {
		// more to write or write zero packet
		this->transferIt = transferIt;
		this->startWrite();

		// not finished yet
		return false;
	}

	// write operation has finished: set number of transferred bytes
	this->p.size = transferIt - this->p.data;

	if (!partial) {
		// enter status stage by sending ZLP of opposite direction
		usbControlStatusOut();
		this->device.controlMode = UsbDevice_USB::Mode::STATUS;
	}

	// push finished buffer to event loop so that ControlBufferBase::handle() gets called from the event loop
	this->device.loop.push(*this);

	// finished
	return true;
}

void UsbDevice_USB::ControlBufferBase::handle() {
	setReady();
}


// BufferBase

UsbDevice_USB::BufferBase::BufferBase(uint8_t *data, int capacity, Endpoint &endpoint)
	: Buffer(data, capacity, endpoint.device.stat)
	, endpoint(endpoint)
{
	endpoint.buffers.add(*this);
}

UsbDevice_USB::BufferBase::~BufferBase() {
}

bool UsbDevice_USB::BufferBase::start(Op op) {
	if (this->st.state != State::READY) {
		assert(this->p.state != State::BUSY);
		return false;
	}

	// check if READ or WRITE flag is set
	assert((op & Op::READ_WRITE) != 0);
	this->op = op;

	// start the transfer
	auto &device = this->endpoint.device;
	this->transferIt = this->p.data;
	if ((op & Op::WRITE) == 0) {
		// read/OUT
		this->transferEnd = this->p.data + this->p.capacity;
		int ep = this->endpoint.outIndex;
		auto &transfer = device.transfers[ep - 1];

		// add to list of pending transfers and start immediately if list was empty
		if (transfer.out.push(nvic::Guard(USB_IRQn), *this)) {
			// check if a packet is already available
			if (transfer.outAvailable) {
				transfer.outAvailable = false;

				// remove from list of pending transfers again if transfer was only one packet
				transfer.out.pop(
					[](BufferBase &buffer) {
						return buffer.finishRead();
					}
				);
			} else if (device.stat == Device::State::READY) {
				// indicate that we want to receive data from the host
				usbReceive(ep);
			}
		}

		// now wait for data arriving from host
	} else {
		// write/IN
		this->transferEnd = this->p.data + this->p.size;
		int ep = this->endpoint.inIndex;
		auto &transfer = device.transfers[ep - 1];

		// add to list of pending transfers and start immediately if list was empty
		if (transfer.in.push(nvic::Guard(USB_IRQn), *this)) {
			if (device.stat == Device::State::READY)
				startWrite();
		}
	}

	// set state
	setBusy();

	return true;
}

bool UsbDevice_USB::BufferBase::cancel() {
	if (this->st.state != State::BUSY)
		return false;
	auto &device = this->endpoint.device;

	// remove from pending transfers even if active
	device.transfers[this->endpoint.inIndex - 1].in.remove(nvic::Guard(USB_IRQn), *this);
	device.transfers[this->endpoint.outIndex - 1].out.remove(nvic::Guard(USB_IRQn), *this);

	// cancel takes effect immediately
	setReady(0);
	return true;
}

bool UsbDevice_USB::BufferBase::finishRead() {
	int ep = this->endpoint.outIndex;
	auto transferIt = this->transferIt;
	auto transferEnd = this->transferEnd;

	int received = ENDPOINT_BUFFER_DESCRIPTORS[ep].rx.transferred();
	int toCopy = std::min(received, int(transferEnd - transferIt));
	usbFinishReceive(ep, transferIt, toCopy);
	transferIt += toCopy;

	if (transferIt < transferEnd && received >= BUFFER_SIZE) {
		// more to read
		this->transferIt = transferIt;

		// wait for next control OUT transfer (host writes)
		usbReceive(ep);

		// not finished yet
		return false;
	}

	// read operation has finished: set number of transferred bytes
	this->p.size = transferIt - this->p.data;

	// push finished buffer to event loop so that ControlBufferBase::handle() gets called from the event loop
	this->endpoint.device.loop.push(*this);

	// finished
	return true;
}

void UsbDevice_USB::BufferBase::startWrite() {
	int ep = this->endpoint.inIndex;
	int toWrite = std::min(int(this->transferEnd - this->transferIt), BUFFER_SIZE);
	usbSend(ep, this->transferIt, toWrite);

	// now wait for USB_EP_CTR_TX
}

// always called from interrupt
bool UsbDevice_USB::BufferBase::finishWrite() {
	int ep = this->endpoint.inIndex;
	int transferred = ENDPOINT_BUFFER_DESCRIPTORS[ep].tx.size;
	auto transferIt = this->transferIt + transferred;
	auto transferEnd = this->transferEnd;

	// check if more to write or a zero packet is needed
	if (transferIt < transferEnd || ((this->op & Op::PARTIAL) == 0 && transferred == BUFFER_SIZE)) {
		// more to write or write zero packet
		this->transferIt = transferIt;
		this->startWrite();

		// not finished yet
		return false;
	}

	// write operation has finished

	if ((this->op & Op::READ) != 0) {
		// read/OUT after write
		auto &device = this->endpoint.device;
		this->transferEnd = this->p.data + this->p.capacity;
		int ep = this->endpoint.outIndex;
		auto &transfer = device.transfers[ep - 1];

		// add to list of pending transfers and start immediately if list was empty
		if (transfer.out.push(*this)) { // disable interrupt not necessary as finishWrite() is always called from interrupt
			// check if a packet is already available
			if (transfer.outAvailable) {
				transfer.outAvailable = false;

				// remove from list of pending transfers again if packet was available
				transfer.out.pop(
					[](BufferBase &buffer) {
						return buffer.finishRead();
					}
				);
			} else if (device.stat == Device::State::READY) {
				// indicate that we want to receive data from the host
				usbReceive(ep);
			}
		}
	} else {
		// set number of transferred bytes
		this->p.size = transferIt - this->p.data;

		// push finished buffer to event loop so that ControlBufferBase::handle() gets called from the event loop
		this->endpoint.device.loop.push(*this);
	}

	// finished
	return true;
}

void UsbDevice_USB::BufferBase::handle() {
	setReady();
}


// Endpoint

UsbDevice_USB::Endpoint::Endpoint(UsbDevice_USB &device, int inIndex, int outIndex, EndpointType type)
	: BufferDevice(device.st.state)
	, device(device), inIndex(inIndex), outIndex(outIndex), type(type)
{
	device.endpoints.add(*this);
}

UsbDevice_USB::Endpoint::~Endpoint() {
}

//StateTasks<const Device::State, Device::Events> &UsbDevice_USB::Endpoint::getStateTasks() {
//	return makeConst(this->device.st);
//}

/*BufferDevice::State UsbDevice_USB::Endpoint::state() {
	return this->device.stat;
}

Awaitable<Device::Condition> UsbDevice_USB::Endpoint::until(Condition condition) {
	// check if IN_* condition is met
	if ((int(condition) >> int(this->device.stat.load())) & 1)
		return {}; // don't wait
	return {this->device.stateTasks, condition};
}*/

int UsbDevice_USB::Endpoint::getBufferCount() {
	return this->buffers.count();
}

UsbDevice_USB::BufferBase &UsbDevice_USB::Endpoint::getBuffer(int index) {
	return this->buffers.get(index);
}

} // namespace coco
#endif

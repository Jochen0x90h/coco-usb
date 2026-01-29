#include "UsbDevice_USBD.hpp"
//#include <coco/debug.hpp>
#include <coco/platform/nvic.hpp>


namespace coco {

UsbDevice_USBD::UsbDevice_USBD(Loop_Queue &loop)
    : UsbDevice(State::OPENING), loop_(loop)
{
    //NRF_POWER->INTENSET = N(POWER_INTENSET_USBDETECTED) | N(POWER_INTENSET_USBREMOVED);

    NRF_USBD->INTENSET = N(USBD_INTENSET_USBRESET, Set)
        | N(USBD_INTENSET_USBEVENT, Set)
        | N(USBD_INTENSET_EP0SETUP, Set)
        | N(USBD_INTENSET_EP0DATADONE, Set)
        | N(USBD_INTENSET_EPDATA, Set);

    NRF_USBD->ENABLE = N(USBD_ENABLE_ENABLE, Enabled);

    // enalbe USB interrupt
    nvic::enable(USBD_IRQn);
}

UsbDevice_USBD::~UsbDevice_USBD() {
}

usb::Setup UsbDevice_USBD::getSetup() {
    auto requestType = usb::RequestType(NRF_USBD->BMREQUESTTYPE);
    uint8_t request = NRF_USBD->BREQUEST;
    uint16_t value = (NRF_USBD->WVALUEH << 8) | NRF_USBD->WVALUEL;
    uint16_t index = (NRF_USBD->WINDEXH << 8) | NRF_USBD->WINDEXL;
    uint16_t length = (NRF_USBD->WLENGTHH << 8) | NRF_USBD->WLENGTHL;
    return {requestType, request, value, index, length};
}

void UsbDevice_USBD::acknowledge() {
    NRF_USBD->TASKS_EP0STATUS = TRIGGER;
}

void UsbDevice_USBD::stall() {
    NRF_USBD->TASKS_EP0STALL = TRIGGER;
}

void UsbDevice_USBD::USBD_IRQHandler() {
    if (NRF_USBD->EVENTS_USBRESET) {
        // clear interrupt flag
        NRF_USBD->EVENTS_USBRESET = 0;

        // change state
        /*if (stat != State::DISABLED) {
            stat = State::DISABLED;

            // resume all coroutines waiting for state change
            stateTasks.doAll();// don't do inside interrupt handler
        }*/
    }
    if (NRF_USBD->EVENTS_USBEVENT) {
        // clear interrupt flag
        NRF_USBD->EVENTS_USBEVENT = 0;

        // check cause
        if (NRF_USBD->EVENTCAUSE & N(USBD_EVENTCAUSE_READY, Ready)) {
            // usb is ready
            NRF_USBD->EVENTCAUSE = N(USBD_EVENTCAUSE_READY, Ready);

            // enable pullup
            NRF_USBD->USBPULLUP = N(USBD_USBPULLUP_CONNECT, Enabled);

            // enable control buffers
            //for (auto &buffer : controlBuffers) {
            //	buffer.setReady(0);
            //}
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
        // received a setup request from host
        // clear interrupt flag
        NRF_USBD->EVENTS_EP0SETUP = 0;

        // setup request
        auto requestType = usb::RequestType(NRF_USBD->BMREQUESTTYPE);
        uint8_t request = NRF_USBD->BREQUEST;
        if (requestType == usb::RequestType::STANDARD_DEVICE_OUT && request == usb::Request::SET_ADDRESS) {
            // set address, handled by hardware
        } else if (requestType == usb::RequestType::STANDARD_DEVICE_OUT && request == usb::Request::SET_CONFIGURATION) {
            // set configuration
            //uint8_t configurationValue = NRF_USBD->WVALUEL;

            // enable endpoints
            int inFlags = 1;
            int outFlags = 1;
            for (auto &endpoint : endpoints_) {
                if (endpoint.inIndex_ > 0)
                    inFlags |= 1 << endpoint.inIndex_;
                if (endpoint.outIndex_ > 0)
                    outFlags |= 1 << endpoint.outIndex_;
            }
            NRF_USBD->EPINEN = inFlags;
            NRF_USBD->EPOUTEN = outFlags;

            // acknowledge the control transfer
            NRF_USBD->TASKS_EP0STATUS = TRIGGER;

            // reset toggles, start OUT and enable buffers
            for (auto &endpoint : endpoints_) {
                NRF_USBD->DTOGGLE = endpoint.inIndex_ | N(USBD_DTOGGLE_IO, In) | N(USBD_DTOGGLE_VALUE, Data0);
                NRF_USBD->DTOGGLE = endpoint.outIndex_ | N(USBD_DTOGGLE_IO, Out) | N(USBD_DTOGGLE_VALUE, Data0);

                // write any value to start receiving OUT transfers into intermediate buffer
                NRF_USBD->SIZE.EPOUT[endpoint.outIndex_] = 0;
            }

            // start IN transfers that are already waiting
            for (int ep = 1; ep < 8; ++ep) {
                auto &transfers = bulkTransfers_[ep - 1];
                auto inBuffer = transfers.in.frontOrNull();
                if (inBuffer != nullptr)
                    inBuffer->writeFirst();
            }

            // set state
            iState_ = State::READY;

            // push this to the event handler queue so that the application gets notified about the sate change in UsbDevice_USB::handle()
            Events e = iEvents_;
            iEvents_ = e | Events::ENTER_READY;
            if (e == Events::NONE)
                loop_.push(*this);
        } else if (requestType == usb::RequestType::STANDARD_INTERFACE_OUT && request == usb::Request::SET_INTERFACE) {
            // set interface (interface index is in setup.index, alternate setting is in setup.value)

            // acknowledge the control transfer
            NRF_USBD->TASKS_EP0STATUS = TRIGGER;
        } else {
            controlMode_ = (requestType & usb::RequestType::DIRECTION_MASK) == usb::RequestType::OUT ? Mode::DATA_OUT : Mode::DATA_IN;

            // push this to the event handler queue so that the control request gets forwarded to the application in UsbDevice_USB::handle()
            Events e = iEvents_;
            iEvents_ = e | Events::REQUEST;
            if (e == Events::NONE)
                loop_.push(*this);
        }
    }

    if (NRF_USBD->EVENTS_EP0DATADONE) {
        // received data from host or sent data to host
        // clear interrupt flag
        NRF_USBD->EVENTS_EP0DATADONE = 0;

        if (controlMode_ == Mode::DATA_IN) {
            // control IN transfer completed
            controlTransfers_.pop(
                [](ControlBufferBase &buffer) {
                    return buffer.writeNext();
                },
                [](ControlBufferBase &next) {
                    // start next buffer, only allowed when previus buffer had PARTIAL flag
                    next.writeFirst();
                }
            );
        } else {
            // control OUT transfer completed
            controlTransfers_.pop(
                [](ControlBufferBase &buffer) {
                    return buffer.readNext();
                },
                [](ControlBufferBase &next) {
                    // start next buffer, only allowed when previus buffer had PARTIAL flag
                }
            );
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
        // clear interrupt flag
        NRF_USBD->EVENTS_EPDATA = 0;

        // get and clear endpoint flags
        uint32_t dataStatus = NRF_USBD->EPDATASTATUS;
        NRF_USBD->EPDATASTATUS = dataStatus;

        for (int ep = 1; ep < 8; ++ep) {
            int inFlag = 1 << ep;
            int outFlag = 0x10000 << ep;

            if (dataStatus & outFlag) {
                // OUT transfer completed (received from host)
                auto &transfer = bulkTransfers_[ep - 1];

                if (transfer.out.pop(
                    [](BufferBase &buffer) {
                        return buffer.readNext();
                    },
                    [](BufferBase &next) {
                        // start next buffer
                    }
                ) == -1) {
                    // store flag that there is a pending OUT packet
                    transfer.outAvailable = true;
                }
            }
            if (dataStatus & inFlag) {
                // IN transfer completed (sent to host)
                auto &transfer = bulkTransfers_[ep - 1];

                transfer.in.pop(
                    [](BufferBase &buffer) {
                        return buffer.writeNext();
                    },
                    [](BufferBase &next) {
                        // start next buffer
                        next.writeFirst();
                    }
                );
            }
        }
    }
}

// called from event loop to notify app about state changes and control requests
void UsbDevice_USBD::handle() {
    State state = iState_;
    Events events = iEvents_.exchange(Events::NONE);

    // set state and resume all coroutines waiting for state change
    st.set(state);
    if ((events & Events::ENTER_ANY) != 0) {
        for (auto &endpoint : endpoints_)
            endpoint.st.set(state).notify(events);
    }
    st.notify(events);
}


// ControlBufferBase

UsbDevice_USBD::ControlBufferBase::ControlBufferBase(uint8_t *data, int capacity, UsbDevice_USBD &device)
    : Buffer(data, capacity, device.iState_)
    , device_(device)
{
    device.controlBuffers_.add(*this);
}

UsbDevice_USBD::ControlBufferBase::~ControlBufferBase() {
}

bool UsbDevice_USBD::ControlBufferBase::start(Op op) {
    if (st.state != State::READY) {
        assert(st.state != State::BUSY);
        return false;
    }

    // check if READ or WRITE flag is set
    assert((op & Op::READ_WRITE) != 0);
    op_ = op;

    // start the transfer
    // note that for read transfers no zero length packet follows when last transfer is 64 bytes
    auto &device = device_;
    transferIt_ = data_;
    transferEnd_ = data_ + size_;
    if ((op & Op::WRITE) == 0) {
        // read/OUT
        if (device.controlMode_ != UsbDevice_USBD::Mode::DATA_OUT) {
            assert(false);
            return false;
        }

        // add to list of pending transfers and start immediately if list was empty
        if (device.controlTransfers_.push(nvic::Guard(USBD_IRQn), *this))
            NRF_USBD->TASKS_EP0RCVOUT = TRIGGER; // -> EP0DATADONE

        // now wait for data arriving from host
    } else {
        // write/IN
        if (device.controlMode_ != UsbDevice_USBD::Mode::DATA_IN) {
            assert(false);
            return false;
        }

        // add to list of pending transfers and start immediately if list was empty
        if (device.controlTransfers_.push(nvic::Guard(USBD_IRQn), *this))
            writeFirst();
    }

    // set state
    setBusy();

    return true;
}

bool UsbDevice_USBD::ControlBufferBase::cancel() {
    if (st.state != State::BUSY)
        return false;
    auto &device = device_;

    // remove from pending transfers even if active
    device.controlTransfers_.remove(nvic::Guard(USBD_IRQn), *this);

    // cancel takes effect immediately
    setReady(0);
    return true;
}

bool UsbDevice_USBD::ControlBufferBase::readNext() {
    int ep = 0;
    auto transferIt = transferIt_;
    auto transferEnd = transferEnd_;

    int received = NRF_USBD->SIZE.EPOUT[ep];
    int toCopy = std::min(received, int(transferEnd - transferIt));
    NRF_USBD->EPOUT[ep].PTR = intptr_t(transferIt);
    NRF_USBD->EPOUT[ep].MAXCNT = toCopy;

    // start DMA transfer from RAM to buffer
    *(volatile uint32_t *)0x40027C1C = 0x00000082;
    NRF_USBD->TASKS_STARTEPOUT[ep] = TRIGGER; // -> ENDEPOUT[i]

    // wait for end of transfer (easy solution and because of errata 199)
    while (!NRF_USBD->EVENTS_ENDEPOUT[ep]);
    NRF_USBD->EVENTS_ENDEPOUT[ep] = 0;
    *(volatile uint32_t *)0x40027C1C = 0x00000000;

    transferIt += toCopy; // NRF_USBD->EPOUT[i].AMOUNT

    if (transferIt < transferEnd && received >= BUFFER_SIZE) {
        // more to read
        transferIt_ = transferIt;

        // wait for next control OUT transfer (host writes)
        NRF_USBD->TASKS_EP0RCVOUT = TRIGGER; // -> EP0DATADONE

        // not finished yet
        return false;
    }

    // read operation has finished: set number of transferred bytes
    size_ = transferIt - data_;

    if ((op_ & Op::PARTIAL) == 0) {
        // enter status stage by sending ZLP of opposite direction (done by hardware)
        NRF_USBD->TASKS_EP0STATUS = TRIGGER;
        device_.controlMode_ = UsbDevice_USBD::Mode::IDLE;
    }

    // push finished buffer to event loop so that ControlBufferBase::handle() gets called from the event loop
    device_.loop_.push(*this);

    // finished
    return true;
}

void UsbDevice_USBD::ControlBufferBase::writeFirst() {
    int ep = 0;
    int toWrite = std::min(int(transferEnd_ - transferIt_), BUFFER_SIZE);
    NRF_USBD->EPIN[ep].PTR = intptr_t(transferIt_);
    NRF_USBD->EPIN[ep].MAXCNT = toWrite;

    // start DMA transfer from RAM to buffer
    *(volatile uint32_t *)0x40027C1C = 0x00000082;
    NRF_USBD->TASKS_STARTEPIN[ep] = TRIGGER; // -> ENDEPIN[i]

    // wait for end of transfer (easy solution and because of errata 199)
    while (!NRF_USBD->EVENTS_ENDEPIN[ep]);
    NRF_USBD->EVENTS_ENDEPIN[ep] = 0;
    *(volatile uint32_t *)0x40027C1C = 0x00000000;

    // now wait for EVENTS_EP0DATADONE
}

bool UsbDevice_USBD::ControlBufferBase::writeNext() {
    int ep = 0;
    int transferred = NRF_USBD->EPIN[ep].AMOUNT;
    auto transferIt = transferIt_ + transferred;
    auto transferEnd = transferEnd_;
    bool partial = (op_ & Op::PARTIAL) != 0;

    // check if more to write or a zero packet is needed
    if (transferIt < transferEnd || (!partial && transferred == BUFFER_SIZE)) {
        // more to write or write zero packet
        transferIt_ = transferIt;
        writeFirst();

        // not finished yet
        return false;
    }

    // write operation has finished: set number of transferred bytes
    size_ = transferIt - data_;

    if (!partial) {
        // enter status stage by sending ZLP of opposite direction (done by hardware)
        NRF_USBD->TASKS_EP0STATUS = TRIGGER;
        device_.controlMode_ = UsbDevice_USBD::Mode::IDLE;
    }

    // push finished buffer to event loop so that ControlBufferBase::handle() gets called from the event loop
    device_.loop_.push(*this);

    // finished
    return true;
}

void UsbDevice_USBD::ControlBufferBase::handle() {
    setReady();
}


// BufferBase

UsbDevice_USBD::BufferBase::BufferBase(uint8_t *data, int capacity, Endpoint &endpoint)
    : Buffer(data, capacity, endpoint.device_.iState_)
    , endpoint_(endpoint)
{
    endpoint.buffers_.add(*this);
}

UsbDevice_USBD::BufferBase::~BufferBase() {
}

bool UsbDevice_USBD::BufferBase::start(Op op) {
    if (st.state != State::READY) {
        assert(st.state != State::BUSY);
        return false;
    }

    // check if READ or WRITE flag is set
    assert((op & Op::READ_WRITE) != 0);
    op_ = op;

    // start the transfer
    auto &device = endpoint_.device_;
    transferIt_ = data_;
    if ((op & Op::WRITE) == 0) {
        // read/OUT
        transferEnd_ = data_ + capacity_;
        int ep = endpoint_.outIndex_;
        auto &transfer = device.bulkTransfers_[ep - 1];

        // add to list of pending transfers and start immediately if list was empty
        if (transfer.out.push(nvic::Guard(USBD_IRQn), *this)) {
            // check if a packet is already available
            if (transfer.outAvailable) {
                transfer.outAvailable = false;

                // remove from list of pending transfers again if transfer was only one packet
                transfer.out.pop(
                    [](BufferBase &buffer) {
                        return buffer.readNext();
                    }
                );
            } else if (device.iState_ == Device::State::READY) {
                // nothing to do, receive starts automatically
            }
        }

        // now wait for data arriving from host
    } else {
        // write/IN
        transferEnd_ = data_ + size_;
        int ep = endpoint_.inIndex_;
        auto &transfer = device.bulkTransfers_[ep - 1];

        // add to list of pending transfers and start immediately if list was empty
        if (transfer.in.push(nvic::Guard(USBD_IRQn), *this)) {
            if (device.iState_ == Device::State::READY)
                writeFirst();
        }
    }

    // set state
    setBusy();

    return true;
}

bool UsbDevice_USBD::BufferBase::cancel() {
    if (st.state != State::BUSY)
        return false;
    auto &device = endpoint_.device_;

    // remove from pending transfers even if active
    device.bulkTransfers_[endpoint_.inIndex_ - 1].in.remove(nvic::Guard(USBD_IRQn), *this);
    device.bulkTransfers_[endpoint_.outIndex_ - 1].out.remove(nvic::Guard(USBD_IRQn), *this);

    // cancel takes effect immediately
    setReady(0);
    return true;
}

bool UsbDevice_USBD::BufferBase::readNext() {
    int ep = endpoint_.outIndex_;
    auto transferIt = transferIt_;
    auto transferEnd = transferEnd_;

    int received = NRF_USBD->SIZE.EPOUT[ep];
    int toCopy = std::min(received, int(transferEnd - transferIt));
    NRF_USBD->EPOUT[ep].PTR = intptr_t(transferIt);
    NRF_USBD->EPOUT[ep].MAXCNT = toCopy;

    // start DMA transfer from RAM to buffer
    *(volatile uint32_t *)0x40027C1C = 0x00000082;
    NRF_USBD->TASKS_STARTEPOUT[ep] = TRIGGER; // -> ENDEPOUT[i]

    // wait for end of transfer (easy solution and because of errata 199)
    while (!NRF_USBD->EVENTS_ENDEPOUT[ep]);
    NRF_USBD->EVENTS_ENDEPOUT[ep] = 0;
    *(volatile uint32_t *)0x40027C1C = 0x00000000;

    transferIt += toCopy; // NRF_USBD->EPOUT[ep].AMOUNT

    if (transferIt < transferEnd && received >= BUFFER_SIZE) {
        // more to read
        transferIt_ = transferIt;

        // not finished yet
        return false;
    }

    // read operation has finished: set number of transferred bytes
    size_ = transferIt - data_;

    // push finished buffer to event loop so that ControlBufferBase::handle() gets called from the event loop
    endpoint_.device_.loop_.push(*this);

    // finished
    return true;
}

void UsbDevice_USBD::BufferBase::writeFirst() {
    int ep = endpoint_.inIndex_;
    int toWrite = std::min(int(transferEnd_ - transferIt_), BUFFER_SIZE);
    NRF_USBD->EPIN[ep].PTR = intptr_t(transferIt_);
    NRF_USBD->EPIN[ep].MAXCNT = toWrite;

    // start DMA transfer from RAM to buffer
    *(volatile uint32_t *)0x40027C1C = 0x00000082;
    NRF_USBD->TASKS_STARTEPIN[ep] = TRIGGER; // -> ENDEPIN[i]

    // wait for end of transfer (easy solution and because of errata 199)
    while (!NRF_USBD->EVENTS_ENDEPIN[ep]);
    NRF_USBD->EVENTS_ENDEPIN[ep] = 0;
    *(volatile uint32_t *)0x40027C1C = 0x00000000;

    // now wait for EVENTS_EPDATA
}

bool UsbDevice_USBD::BufferBase::writeNext() {
    int ep = endpoint_.inIndex_;
    int transferred = NRF_USBD->EPIN[ep].AMOUNT;
    auto transferIt = transferIt_ + transferred;
    auto transferEnd = transferEnd_;

    // check if more to write or a zero packet is needed
    if (transferIt < transferEnd || ((op_ & Op::PARTIAL) == 0 && transferred == BUFFER_SIZE)) {
        // more to write or write zero packet
        transferIt_ = transferIt;
        writeFirst();

        // not finished yet
        return false;
    }

    // write operation has finished

    if ((op_ & Op::READ) != 0) {
        // read/OUT after write
        auto &device = endpoint_.device_;
        transferEnd_ = data_ + capacity_;
        int ep = endpoint_.outIndex_;
        auto &transfer = device.bulkTransfers_[ep - 1];

        // add to list of pending transfers and start immediately if list was empty
        if (transfer.out.push(*this)) { // disable interrupt not necessary as writeNext() is always called from interrupt
            // check if a packet is already available
            if (transfer.outAvailable) {
                transfer.outAvailable = false;

                // remove from list of pending transfers again if packet was available
                transfer.out.pop(
                    [](BufferBase &buffer) {
                        return buffer.readNext();
                    }
                );
            } else if (device.iState_ == Device::State::READY) {
                // nothing to do, receive starts automatically
            }
        }
    } else {
        // set number of transferred bytes
        size_ = transferIt - data_;

        // push finished buffer to event loop so that ControlBufferBase::handle() gets called from the event loop
        endpoint_.device_.loop_.push(*this);
    }

    // finished
    return true;
}

void UsbDevice_USBD::BufferBase::handle() {
    setReady();
}


// Endpoint

UsbDevice_USBD::Endpoint::Endpoint(UsbDevice_USBD &device, int inIndex, int outIndex)
    : BufferDevice(device.st.state)
    , device_(device), inIndex_(inIndex), outIndex_(outIndex)
{
    device.endpoints_.add(*this);
}

UsbDevice_USBD::Endpoint::~Endpoint() {
}

int UsbDevice_USBD::Endpoint::getBufferCount() {
    return buffers_.count();
}

UsbDevice_USBD::BufferBase &UsbDevice_USBD::Endpoint::getBuffer(int index) {
    return buffers_.get(index);
}

} // namespace coco

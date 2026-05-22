#include "UsbDevice_USBD.hpp"
#include <coco/convert.hpp>
#include <coco/debug.hpp>
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

            for (auto &endpoint : endpoints_) {
                if (endpoint.inIndex_ > 0) {
                    // enable
                    NRF_USBD->EPINEN = NRF_USBD->EPINEN | (1 << endpoint.inIndex_);

                    // reset toggle
                    NRF_USBD->DTOGGLE = endpoint.inIndex_ | N(USBD_DTOGGLE_IO, In) | N(USBD_DTOGGLE_VALUE, Data0);
                }
                if (endpoint.outIndex_ > 0) {
                    // enable
                    NRF_USBD->EPOUTEN = NRF_USBD->EPOUTEN | (1 << endpoint.outIndex_);

                    // reset toggle
                    NRF_USBD->DTOGGLE = endpoint.outIndex_ | N(USBD_DTOGGLE_IO, Out) | N(USBD_DTOGGLE_VALUE, Data0);

                    // write any value to start receiving OUT transfers into intermediate buffer
                    NRF_USBD->SIZE.EPOUT[endpoint.outIndex_] = 0;
                }
            }

            // acknowledge the control transfer
            NRF_USBD->TASKS_EP0STATUS = TRIGGER;

            // set state
            //iState_ = State::READY;

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
            auto buffer = controlTransfers_.popIf(
                [](auto &buffer) {
                    // returns false to reject the pop if not finished yet
                    return buffer.writeBuffer();
                },
                [](auto &next) {
                    // start next buffer, only allowed when previus buffer had PARTIAL flag
                    next.writeBuffer();
                });
            if (buffer != nullptr) {
                // push finished buffer to event loop so that ControlBufferBase::handle() gets called from the event loop
                loop_.push(*buffer);
            }
        } else {
            // control OUT transfer completed
            auto buffer = controlTransfers_.popIf(
                [](auto &buffer) {
                    // returns false to reject the pop if not finished yet
                    return buffer.readBuffer();
                },
                [](auto &next) {
                    // start next buffer, only allowed when previus buffer had PARTIAL flag
                    next.readBuffer();
                });
            if (buffer != nullptr) {
                // push finished buffer to event loop so that ControlBufferBase::handle() gets called from the event loop
                loop_.push(*buffer);
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
                auto &transfer = transfers_[ep - 1];

                transfer.outAvailable = transfer.out.empty();
                auto buffer = transfer.out.popIf(
                    [](BufferBase &buffer) {
                        // returns false to reject the pop if not finished yet
                        return buffer.readBuffer();
                    }//,
                    //[](BufferBase &next) {
                        // nothing to do, receive into hardware buffer starts automatically
                    //},
                    );
                if (buffer != nullptr) {
                    // push finished buffer to event loop so that ControlBufferBase::handle() gets called from the event loop
                    loop_.push(*buffer);
                }
            }
            if (dataStatus & inFlag) {
                // IN transfer completed (sent to host)
                auto &transfer = transfers_[ep - 1];

                auto b = transfer.in.popIf(
                    [](BufferBase &buffer) {
                        // returns false to reject the pop if not finished yet
                        return buffer.writeBuffer();
                    },
                    [](BufferBase &next) {
                        // start next buffer
                        next.writeBuffer();
                    });
                if (b != nullptr) {
                    auto &buffer = *b;
                    if ((buffer.op_ & BufferBase::Op::READ) != 0) {
                        // read/OUT after write
                        buffer.transferEnd_ = buffer.data_ + buffer.capacity_;
                        int ep = buffer.endpoint_.outIndex_;
                        auto &transfer = transfers_[ep - 1];

                        // add to list of pending transfers and start immediately if list was empty
                        if (transfer.out.push(buffer)) { // disable interrupt not necessary as writeNext() is always called from interrupt
                            // check if a packet is already available
                            if (transfer.outAvailable) {
                                transfer.outAvailable = false;

                                // remove from list of pending transfers again if packet was available
                                if (transfer.out.popIf(
                                    [](BufferBase &buffer) {
                                        // returns false to reject the pop if not finished yet
                                        return buffer.readBuffer();
                                    }) != nullptr)
                                {
                                    // push finished buffer to event loop so that ControlBufferBase::handle() gets called from the event loop
                                    loop_.push(buffer);
                                }
                            } else {
                                // nothing to do, receive into hardware buffer starts automatically
                            }
                        }
                    } else {
                        setSuccess();

                        // push finished buffer to event loop so that ControlBufferBase::handle() gets called from the event loop
                        loop_.push(buffer);
                    }
                }
            }
        }
    }
}

// called from event loop to notify app about state changes and control requests
void UsbDevice_USBD::onCompletion() {
    Events events = iEvents_.exchange(Events::NONE);

    if ((events & Events::ENTER_READY) != 0) {
        //debug::out << "ready\n";
        state_ = State::READY;

        // note that the control buffers are always READY so that control requests can be handled

        for (auto &endpoint : endpoints_) {
            endpoint.state_ = State::READY;
            for (auto &buffer : endpoint.buffers_) {
                buffer.state_ = BufferBase::State::READY;
                buffer.notify(BufferBase::Events::ENTER_READY);
            }
            endpoint.notify(Events::ENTER_READY);
        }
    }

    // notify REQUEST or ENTER_READY events for the USB device to the application
    notify(events);
/*

    State state = iState_;
    Events events = iEvents_.exchange(Events::NONE);

    // set state and resume all coroutines waiting for state change
    st.set(state);
    if ((events & Events::ENTER_ANY) != 0) {
        for (auto &endpoint : endpoints_)
            endpoint.st.set(state).notify(events);
    }
    st.notify(events);*/
}


// ControlBufferBase

UsbDevice_USBD::ControlBufferBase::ControlBufferBase(uint8_t *data, int capacity, UsbDevice_USBD &device)
    : Buffer(data, capacity, Buffer::State::READY) // control buffers are always READY
    , device_(device)
{
    device.controlBuffers_.add(*this);
}

UsbDevice_USBD::ControlBufferBase::~ControlBufferBase() {
}

bool UsbDevice_USBD::ControlBufferBase::start() {
    if (state_ != State::READY) {
        assert(false);
        setError(std::errc::resource_unavailable_try_again);
        return false;
    }
    if ((op_ & Op::READ_WRITE) == 0 || size_ == 0) {
        setSuccess();
        return false;
    }

    // start the transfer
    // note that for read transfers no zero length packet follows when last transfer is 64 bytes
    auto &device = device_;
    transferIt_ = data_;
    transferEnd_ = data_ + size_;
    if ((op_ & Op::WRITE) == 0) {
        // read/OUT
        if (device.controlMode_ != UsbDevice_USBD::Mode::DATA_OUT) {
            assert(false);
            return false;
        }

        // add to list of pending transfers and start immediately if list was empty
        if (device.controlTransfers_.guardedPush(nvic::Guard(USBD_IRQn), *this))
            NRF_USBD->TASKS_EP0RCVOUT = TRIGGER; // -> EP0DATADONE

        // now wait for data arriving from host
    } else {
        // write/IN
        if (device.controlMode_ != UsbDevice_USBD::Mode::DATA_IN) {
            assert(false);
            return false;
        }

        // add to list of pending transfers and start immediately if list was empty
        if (device.controlTransfers_.guardedPush(nvic::Guard(USBD_IRQn), *this))
            writeBuffer();
    }

    // set state
    setBusy();

    return true;
}

bool UsbDevice_USBD::ControlBufferBase::cancel() {
    if (state_ != State::BUSY)
        return false;
    auto &device = device_;

    // remove from pending transfers even if active
    device.controlTransfers_.guardedRemove(nvic::Guard(USBD_IRQn), *this);

    // cancel takes effect immediately
    setError(std::errc::operation_canceled);
    setReady();
    return true;
}

bool UsbDevice_USBD::ControlBufferBase::readBuffer() {
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

    setSuccess();

    // finished
    return true;
}

bool UsbDevice_USBD::ControlBufferBase::writeBuffer() {
    bool partial = (op_ & Op::PARTIAL) != 0;

    auto transferIt = transferIt_;
    int toWrite = std::min(int(transferEnd_ - transferIt), BUFFER_SIZE);
    if (toWrite > 0 || (partial && toWrite == 0)) {
        // write data or ZLP
        int ep = 0;

        NRF_USBD->EPIN[ep].PTR = intptr_t(transferIt);
        NRF_USBD->EPIN[ep].MAXCNT = toWrite;

        // start DMA transfer from RAM to buffer
        *(volatile uint32_t *)0x40027C1C = 0x00000082;
        NRF_USBD->TASKS_STARTEPIN[ep] = TRIGGER; // -> ENDEPIN[i]

        // wait for end of transfer (easy solution and because of errata 199)
        while (!NRF_USBD->EVENTS_ENDEPIN[ep]);
        NRF_USBD->EVENTS_ENDEPIN[ep] = 0;
        *(volatile uint32_t *)0x40027C1C = 0x00000000;

        transferIt_ = transferIt + BUFFER_SIZE;

        // not finished yet -> EVENTS_EP0DATADONE
        return false;
    }

    // write operation has finished

    if (!partial) {
        // enter status stage by sending ZLP of opposite direction (done by hardware)
        NRF_USBD->TASKS_EP0STATUS = TRIGGER;
        device_.controlMode_ = UsbDevice_USBD::Mode::IDLE;
    }

    setSuccess();

    // finished
    return true;
}

void UsbDevice_USBD::ControlBufferBase::onCompletion() {
    setReady();
}


// BufferBase

UsbDevice_USBD::BufferBase::BufferBase(uint8_t *data, int capacity, Endpoint &endpoint)
    : Buffer(data, capacity, endpoint.device_.state_)
    , endpoint_(endpoint)
{
    endpoint.buffers_.add(*this);
}

UsbDevice_USBD::BufferBase::~BufferBase() {
}

bool UsbDevice_USBD::BufferBase::start() {
    if (state_ != State::READY) {
        assert(false);
        setError(std::errc::resource_unavailable_try_again);
        return false;
    }

    // sending/receiving zero length packets is allowed
    if ((op_ & Op::READ_WRITE) == 0) {
        setSuccess();
        return false;
    }

    // start the transfer
    auto &device = endpoint_.device_;
    transferIt_ = data_;
    transferEnd_ = data_ + size_;
    if ((op_ & Op::WRITE) == 0) {
        // read/OUT
        int ep = endpoint_.outIndex_;
        auto &transfer = device.transfers_[ep - 1];

        // add to list of pending transfers and start immediately if list was empty
        if (transfer.out.guardedPush(nvic::Guard(USBD_IRQn), *this)) {
            // check if a packet is already available
            if (transfer.outAvailable) {
                transfer.outAvailable = false;

                // remove from list of pending transfers again if transfer was only one packet
                if (transfer.out.popIf(
                    [](BufferBase &buffer) {
                        // returns false to reject the pop if not finished yet
                        return buffer.readBuffer();
                    }) != nullptr)
                {
                    // push finished buffer to event loop so that ControlBufferBase::handle() gets called from the event loop
                    device.loop_.push(*this);
                }
            } else {
                // nothing to do, receive into hardware buffer starts automatically
            }
        }

        // now wait for data arriving from host
    } else {
        // write/IN
        int ep = endpoint_.inIndex_;
        auto &transfer = device.transfers_[ep - 1];

        // add to list of pending transfers and start immediately if list was empty
        if (transfer.in.guardedPush(nvic::Guard(USBD_IRQn), *this)) {
            writeBuffer();
        }
    }

    // set state
    setBusy();

    return true;
}

bool UsbDevice_USBD::BufferBase::cancel() {
    if (state_ != State::BUSY)
        return false;
    auto &device = endpoint_.device_;

    // read/out: can remove even if active because the next transfer will go into the next buffer or set outAvailable
    device.transfers_[endpoint_.outIndex_ - 1].out.guardedRemove(nvic::Guard(USBD_IRQn), *this);

    // write/in: can remove even if active because next transfer will call write() on next buffer
    device.transfers_[endpoint_.inIndex_ - 1].in.guardedRemove(nvic::Guard(USBD_IRQn), *this);

    // cancel takes effect immediately
    setError(std::errc::operation_canceled);
    setReady();
    return true;
}

bool UsbDevice_USBD::BufferBase::readBuffer() {
    int ep = endpoint_.outIndex_;
    auto transferIt = transferIt_;
    auto transferEnd = transferEnd_;

    int received = NRF_USBD->SIZE.EPOUT[ep];
    int toCopy = std::min(received, int(transferEnd - transferIt));
//debug::out << "received " << dec(received) << '\n';
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
    setSuccess(transferIt - data_);
//debug::out << "success\n";

    // finished
    return true;
}

bool UsbDevice_USBD::BufferBase::writeBuffer() {
    auto transferIt = transferIt_;
    int toWrite = std::min(int(transferEnd_ - transferIt), BUFFER_SIZE);
    if (toWrite > 0 || ((op_ & Op::PARTIAL) == 0 && toWrite == 0)) {
        // write data or ZLP
        int ep = endpoint_.inIndex_;

        NRF_USBD->EPIN[ep].PTR = intptr_t(transferIt);
        NRF_USBD->EPIN[ep].MAXCNT = toWrite;

        // start DMA transfer from RAM to buffer
        *(volatile uint32_t *)0x40027C1C = 0x00000082;
        NRF_USBD->TASKS_STARTEPIN[ep] = TRIGGER; // -> ENDEPIN[i]

        // wait for end of transfer (easy solution and because of errata 199)
        while (!NRF_USBD->EVENTS_ENDEPIN[ep]);
        NRF_USBD->EVENTS_ENDEPIN[ep] = 0;
        *(volatile uint32_t *)0x40027C1C = 0x00000000;

        transferIt_ = transferIt + BUFFER_SIZE;

        // not finished yet
        return false;
    }

    // write operation has finished (setSuccess is called after check for read-after-write)


    // write finished
    return true;

}

void UsbDevice_USBD::BufferBase::onCompletion() {
    setReady();
}


// Endpoint

UsbDevice_USBD::Endpoint::Endpoint(UsbDevice_USBD &device, int inIndex, int outIndex)
    : BufferDevice(device.state_)
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

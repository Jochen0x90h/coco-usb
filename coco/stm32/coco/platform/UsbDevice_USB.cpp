#include "UsbDevice_USB.hpp"
#include <coco/convert.hpp>
#include <coco/debug.hpp>
#include <coco/platform/nvic.hpp>


/*
    Glossary
        ZLP: Zero Length Packet

    Packet Memory layout
        Table of buffer descriptors (BufferDescriptor[8])
        Buffers for endpoint 0
            RX Buffer of 64 bytes
            TX Buffer of 64 bytes
        Buffers for endpoint 1
        Bbuffers for endpoint 2
        ...
        Buffers for endpoint 7
*/

#ifdef HAVE_USBD

namespace coco {

UsbDevice_USB::UsbDevice_USB(Loop_Queue &loop)
    : UsbDevice(State::OPENING)
    , loop_(loop)
{
    //debug::out << "UsbDevice_USB\n";

    usbd::enableClock()
        .clear(usbd::Status::ALL)
        .enable(usbd::Interrupt::RESET | usbd::Interrupt::CORRECT_TRANSFER);

    // enable USB interrupt
    nvic::setPriority(usbd::irq, nvic::Priority::HIGH);
    nvic::enable(usbd::irq);
    //debug::out << "UsbDevice_USB2\n";
}

UsbDevice_USB::~UsbDevice_USB() {
}

usb::Setup UsbDevice_USB::getSetup() {
    auto usb = usbd::instance();
    return usb.getSetup<usb::Setup, usb::RequestType>();
}

void UsbDevice_USB::acknowledge() {
    auto usb = usbd::instance();

    // acknowledge using ZLP of opposite direction
    if (controlMode_ == Mode::DATA_IN)
        usb.controlStatusOut();
    else if (controlMode_ == Mode::DATA_OUT)
        usb.controlStatusIn();
    else
        assert(false);
}

void UsbDevice_USB::stall() {
    auto usb = usbd::instance();

    usb.controlStall();
}

void UsbDevice_USB::USB_IRQHandler() {
    //debug::out << "irq\n";
    auto usb = usbd::instance();

    auto status = usb.status();
    if ((status & usbd::Status::RESET) != 0) {
        // device was connected to the host
        //debug::out << "connected\n";
        usbd::BufferCapacity bufferCapacity[1] = {{usbd::RxCapacity::_64, 64}};
        usb.configure(bufferCapacity);
        usb.reset();
    }

    auto status0 = usb.endpointStatus(0);
    if ((status0 & usbd::EndpointStatus::RX) != 0) {
        //debug::out << "rx\n";
        // received a packet from the host on endpoint 0
        usb.rxAck(0);

        if ((status0 & usbd::EndpointStatus::SETUP) != 0) {
            // received a setup request from host
            auto [requestType, request] = usb.getSetupRequest<usb::RequestType>();
            //debug::out << "setup " << hex(rt) << ' ' << hex(request) << '\n';

            if (requestType == usb::RequestType::STANDARD_DEVICE_OUT && request == usb::Request::SET_ADDRESS) {
                // address is value of setup packet, keep in control receive buffer
                //debug::out << "set address\n";

                // enter status stage by sending ZLP of opposite direction
                usb.controlStatusIn();
                controlMode_ = Mode::SET_ADDRESS;
            } else if (requestType == usb::RequestType::STANDARD_DEVICE_OUT && request == usb::Request::SET_CONFIGURATION) {
                // set configuration
                //debug::out << "set configuration\n";

                // configure endpoints
                usbd::BufferCapacity bufferCapacities[8] = {};
                for (auto &endpoint : endpoints_) {
                    if (endpoint.inIndex_ > 0)
                        usb.txInit(endpoint.inIndex_, endpoint.type_);
                    if (endpoint.outIndex_ > 0)
                        usb.rxInit(endpoint.outIndex_, endpoint.type_);

                    // outIndex_ or inIndex_ are zero when invalid
                    bufferCapacities[endpoint.outIndex_].rx = usbd::RxCapacity::_64;
                    bufferCapacities[endpoint.inIndex_].tx = 64;
                }
                bufferCapacities[0].rx = usbd::RxCapacity::_64;
                bufferCapacities[0].tx = 64;
                usb.configure(bufferCapacities);

                // enter status stage by sending ZLP of opposite direction
                usb.controlStatusIn();
                controlMode_ = Mode::STATUS;

                // start transfers that are already waiting
                for (int ep = 1; ep < 8; ++ep) {
                    auto &transfers = transfers_[ep - 1];
                    auto inBuffer = transfers.in.frontOrNull();
                    if (inBuffer != nullptr)
                        inBuffer->writeFirst();
                    auto outBuffer = transfers.out.frontOrNull();
                    if (outBuffer != nullptr)
                        usb.rxStart(ep);
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

                // enter status stage by sending ZLP of opposite direction
                usb.controlStatusIn();
            } else {
                controlMode_ = (requestType & usb::RequestType::DIRECTION_MASK) == usb::RequestType::OUT ? Mode::DATA_OUT : Mode::DATA_IN;

                // push this to the event handler queue so that the control request gets forwarded to the application in UsbDevice_USB::handle()
                Events e = iEvents_;
                iEvents_ = e | Events::REQUEST;
                if (e == Events::NONE)
                    loop_.push(*this);
            }
        } else {
            // received data from host
            switch (controlMode_) {
            case Mode::DATA_OUT:
                // control OUT transfer completed
                controlTransfers_.pop(
                    [](ControlBufferBase &buffer) {
                        // returns false if not finished yet
                        return buffer.readNext();
                    },
                    [&usb](ControlBufferBase &next) {
                        // start next buffer, only allowed when previus buffer had PARTIAL flag
                        assert(controlMode_ == Mode::DATA_OUT);
                        usb.rxStart(0);//usbReceive(0); // start read
                    }
                );
                break;
            //case Mode::STATUS_IN:
            default:
                // status stage of IN control transfer completed
                //debug::setGreen();
                controlMode_ = Mode::IDLE;
                usb.controlStall();
                break;
            }
        }
    }
    if ((status0 & usbd::EndpointStatus::TX) != 0) {
        // sent data to host
        usb.txAck(0);

        switch (controlMode_) {
        case Mode::DATA_IN:
            // control IN transfer completed
            controlTransfers_.pop(
                [](ControlBufferBase &buffer) {
                    return buffer.writeNext();
                },
                [](ControlBufferBase &next) {
                    // start next buffer, only allowed when previus buffer had PARTIAL flag
                    assert(controlMode_ == Mode::DATA_IN);
                    next.writeFirst();
                }
            );
            break;
        case Mode::SET_ADDRESS:
            // status stage of "set address" OUT control transfer complete, now set address
            {
                //int address =
                usb.applyAddress();
                //debug::out << "set address " << dec(address) << '\n';
            }
            // fall through
        default:
            // status stage of OUT control transfer complete
            controlMode_ = Mode::IDLE;
            usb.controlStall();
            break;
        }
    }

    for (int ep = 1; ep < 8; ++ep) {
        auto statusN = usb.endpointStatus(ep);
        if ((statusN & usbd::EndpointStatus::RX) != 0) {
            // OUT transfer completed (received from host)
            usb.rxAck(ep);
            auto &transfer = transfers_[ep - 1];

            if (transfer.out.pop(
                [](BufferBase &buffer) {
                    // returns false if not finished yet
                    return buffer.readNext();
                },
                [&usb, ep](BufferBase &next) {
                    // start next buffer
                    usb.rxStart(ep); // start read
                }
            ) == -1) {
                // store flag that there is a pending OUT packet
                transfer.outAvailable = true;
            }
        }
        if ((statusN & usbd::EndpointStatus::TX) != 0) {
            // IN transfer completed (sent to host)
            usb.txAck(ep);
            auto &transfer = transfers_[ep - 1];

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

// called from event loop to notify app about state changes and control requests
void UsbDevice_USB::handle() {
    /*nvic::disable(USB_IRQn);
    State state = iState_;
    Events events = iEvents_;
    iEvents_ = Events::NONE;
    nvic::enable(USB_IRQn);*/
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

UsbDevice_USB::ControlBufferBase::ControlBufferBase(uint8_t *data, int capacity, UsbDevice_USB &device)
    : Buffer(data, capacity, device.iState_)
    , device_(device)
{
    device.controlBuffers_.add(*this);
}

UsbDevice_USB::ControlBufferBase::~ControlBufferBase() {
}

bool UsbDevice_USB::ControlBufferBase::start(Op op) {
    if (st.state != State::READY) {
        assert(st.iState_ != State::BUSY);
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
        if (device.controlMode_ != UsbDevice_USB::Mode::DATA_OUT) {
            assert(false);
            return false;
        }

        // add to list of pending transfers and start immediately if list was empty
        if (device.controlTransfers_.push(nvic::Guard(usbd::irq), *this))
            usbd::instance().rxStart(0);

        // now wait for data arriving from host
    } else {
        // write/IN
        if (device.controlMode_ != UsbDevice_USB::Mode::DATA_IN) {
            assert(false);
            return false;
        }

        // add to list of pending transfers and start immediately if list was empty
        if (device.controlTransfers_.push(nvic::Guard(usbd::irq), *this))
            writeFirst();
    }

    // set state
    setBusy();

    return true;
}

bool UsbDevice_USB::ControlBufferBase::cancel() {
    if (st.state != State::BUSY)
        return false;
    auto &device = device_;

    // remove from pending transfers even if active
    device.controlTransfers_.remove(nvic::Guard(usbd::irq), *this);

    // cancel takes effect immediately
    setReady(0);
    return true;
}

bool UsbDevice_USB::ControlBufferBase::readNext() {
    auto usb = usbd::instance();
    int ep = 0;
    auto transferIt = transferIt_;
    auto transferEnd = transferEnd_;

    int copied = usb.rx(ep, (uint32_t *)transferIt, int(transferEnd - transferIt));
    transferIt += copied;

    if (transferIt < transferEnd && copied >= BUFFER_SIZE) {
        // more to read
        transferIt_ = transferIt;

        // wait for next control OUT transfer (host writes)
        usb.rxStart(ep);

        // not finished yet
        return false;
    }

    // read operation has finished: set number of transferred bytes
    size_ = transferIt - data_;

    if ((op_ & Op::PARTIAL) == 0) {
        // enter status stage by sending ZLP of opposite direction
        usb.controlStatusIn();
        device_.controlMode_ = UsbDevice_USB::Mode::STATUS;
    }

    // push finished buffer to event loop so that ControlBufferBase::handle() gets called from the event loop
    device_.loop_.push(*this);

    // finished
    return true;
}

void UsbDevice_USB::ControlBufferBase::writeFirst() {
    auto usb = usbd::instance();
    int ep = 0;
    int toWrite = std::min(int(transferEnd_ - transferIt_), BUFFER_SIZE);
    //debug::out << "tx " << dec(toWrite) << '\n';
    usb.tx(ep, (uint32_t *)transferIt_, toWrite);

    // now wait for interrupt
}

bool UsbDevice_USB::ControlBufferBase::writeNext() {
    auto usb = usbd::instance();
    auto transferIt = transferIt_;
    auto transferEnd = transferEnd_;
    int transferred = std::min(int(transferEnd - transferIt), BUFFER_SIZE);
    transferIt += transferred;
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
        // enter status stage by sending ZLP of opposite direction
        usb.controlStatusOut();
        device_.controlMode_ = UsbDevice_USB::Mode::STATUS;
    }

    // push finished buffer to event loop so that ControlBufferBase::handle() gets called from the event loop
    device_.loop_.push(*this);

    // finished
    return true;
}

void UsbDevice_USB::ControlBufferBase::handle() {
    setReady();
}


// BufferBase

UsbDevice_USB::BufferBase::BufferBase(uint8_t *data, int capacity, Endpoint &endpoint)
    : Buffer(data, capacity, endpoint.device_.iState_)
    , endpoint_(endpoint)
{
    endpoint.buffers_.add(*this);
}

UsbDevice_USB::BufferBase::~BufferBase() {
}

bool UsbDevice_USB::BufferBase::start(Op op) {
    if (st.state != State::READY) {
        assert(st.state != State::BUSY);
        return false;
    }
    auto usb = usbd::instance();

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
        auto &transfer = device.transfers_[ep - 1];

        // add to list of pending transfers and start immediately if list was empty
        if (transfer.out.push(nvic::Guard(usbd::irq), *this)) {
            // check if a packet is already available
            if (transfer.outAvailable) {
                transfer.outAvailable = false;

                // remove from list of pending transfers again if transfer was only one packet
                transfer.out.pop(
                    [](BufferBase &buffer) {
                        // returns false if not finished yet
                        return buffer.readNext();
                    }
                );
            } else if (device.iState_ == Device::State::READY) {
                // indicate that we want to receive data from the host
                usb.rxStart(ep);
            }
        }

        // now wait for data arriving from host
    } else {
        // write/IN
        transferEnd_ = data_ + size_;
        int ep = endpoint_.inIndex_;
        auto &transfer = device.transfers_[ep - 1];

        // add to list of pending transfers and start immediately if list was empty
        if (transfer.in.push(nvic::Guard(usbd::irq), *this)) {
            if (device.iState_ == Device::State::READY)
                writeFirst();
        }
    }

    // set state
    setBusy();

    return true;
}

bool UsbDevice_USB::BufferBase::cancel() {
    if (st.state != State::BUSY)
        return false;
    auto &device = endpoint_.device_;

    // remove from pending transfers even if active
    device.transfers_[endpoint_.inIndex_ - 1].in.remove(nvic::Guard(usbd::irq), *this);
    device.transfers_[endpoint_.outIndex_ - 1].out.remove(nvic::Guard(usbd::irq), *this);

    // cancel takes effect immediately
    setReady(0);
    return true;
}

bool UsbDevice_USB::BufferBase::readNext() {
    auto usb = usbd::instance();
    int ep = endpoint_.outIndex_;
    auto transferIt = transferIt_;
    auto transferEnd = transferEnd_;

    int copied = usb.rx(ep, (uint32_t *)transferIt, int(transferEnd - transferIt));
    transferIt += copied;

    if (transferIt < transferEnd && copied >= BUFFER_SIZE) {
        // more to read
        transferIt_ = transferIt;

        // wait for next OUT transfer (host writes)
        usb.rxStart(ep);

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

void UsbDevice_USB::BufferBase::writeFirst() {
    auto usb = usbd::instance();
    int ep = endpoint_.inIndex_;
    int toWrite = std::min(int(transferEnd_ - transferIt_), BUFFER_SIZE);
    usb.tx(ep, (uint32_t *)transferIt_, toWrite);

    // now wait for interrupt
}

// always called from interrupt
bool UsbDevice_USB::BufferBase::writeNext() {
    auto usb = usbd::instance();
    auto transferIt = transferIt_;
    auto transferEnd = transferEnd_;
    int transferred = std::min(int(transferEnd - transferIt), BUFFER_SIZE);
    transferIt += transferred;

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
        auto &transfer = device.transfers_[ep - 1];

        // add to list of pending transfers and start immediately if list was empty
        if (transfer.out.push(*this)) { // disable interrupt not necessary as writeNext() is always called from interrupt
            // check if a packet is already available
            if (transfer.outAvailable) {
                transfer.outAvailable = false;

                // remove from list of pending transfers again if packet was available
                transfer.out.pop(
                    [](BufferBase &buffer) {
                        // returns false if not finished yet
                        return buffer.readNext();
                    }
                );
            } else if (device.iState_ == Device::State::READY) {
                // indicate that we want to receive data from the host
                usb.rxStart(ep);
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

void UsbDevice_USB::BufferBase::handle() {
    setReady();
}


// Endpoint

UsbDevice_USB::Endpoint::Endpoint(UsbDevice_USB &device, int inIndex, int outIndex, usbd::EndpointType type)
    : BufferDevice(device.st.state)
    , device_(device), inIndex_(inIndex), outIndex_(outIndex), type_(type)
{
    device.endpoints_.add(*this);
}

UsbDevice_USB::Endpoint::~Endpoint() {
}

int UsbDevice_USB::Endpoint::getBufferCount() {
    return buffers_.count();
}

UsbDevice_USB::BufferBase &UsbDevice_USB::Endpoint::getBuffer(int index) {
    return buffers_.get(index);
}

} // namespace coco

#endif // HAVE_USBD

#include "UsbDevice_cout.hpp"
#include <coco/convert.hpp>
#include <coco/debug.hpp>
#include <iostream>
#include <iomanip>


namespace coco {

UsbDevice_cout::UsbDevice_cout(Loop_native &loop)
    : UsbDevice(State::OPENING)
    , loop_(loop), callback_(makeCallback<UsbDevice_cout, &UsbDevice_cout::handle>(this))
{
    // start coroutine
    readDescriptors();
}

usb::Setup UsbDevice_cout::getSetup() {
    return setup_;
}

void UsbDevice_cout::acknowledge() {
}

void UsbDevice_cout::stall() {
}

Coroutine UsbDevice_cout::readDescriptors() {
    co_await loop_.yield();

    // get device descriptor
    setup_ = {
        usb::RequestType::STANDARD_DEVICE_IN,
        uint8_t(usb::Request::GET_DESCRIPTOR),
        uint16_t(int(usb::DescriptorType::DEVICE) << 8),
        0,
        sizeof(usb::DeviceDescriptor)};
    st.notify(Events::REQUEST);
    co_await loop_.yield();

    // get WinUSB descriptor
    setup_ = {
        usb::RequestType::VENDOR_DEVICE_IN,
        uint8_t(6),
        0,
        0x07,
        256};
    st.notify(Events::REQUEST);
    co_await loop_.yield();

    // set ready and resume all coroutines waiting for state change
    st.set(State::READY);
    for (auto &endpoint : endpoints_) {
        endpoint.st.set(State::READY);
        for (auto &buffer : endpoint.buffers_) {
            buffer.st.set(Buffer::State::READY).notify(Buffer::Events::ENTER_READY);
        }
        endpoint.st.notify(Events::ENTER_READY);
    }
    st.notify(Events::ENTER_READY);

    // invoke handle() when transfers are pending
    if (!transfers_.empty())
        loop_.invoke(callback_);
}

void UsbDevice_cout::handle() {
    // check if there is a pending control transfer
    for (auto &buffer : controlTransfers_) {
        auto &setup = setup_;

        if (setup.bmRequestType == usb::RequestType::STANDARD_DEVICE_IN
            && setup.bRequest == usb::Request::GET_DESCRIPTOR
            && usb::DescriptorType(setup.wValue >> 8) == usb::DescriptorType::DEVICE)
        {
            auto &deviceDescriptor = buffer.value<usb::DeviceDescriptor>();
            text_ = deviceDescriptor.bDeviceProtocol == 1;
        }

        int size = std::min(int(setup.wLength), buffer.size());
        debug::out << "R " << hex(setup.bmRequestType) << ' ' << hex(setup.bRequest) << ' ' << hex(setup.wValue) << ' ' << hex(setup.wIndex) << ' ' << hex(size) << '\n';

        for (int i = 0; i < size; ++i) {
            debug::out << hex(buffer.data()[i]) << ' ';
        }
        debug::out << '\n';

        // remove control buffer from controlTransfers and set ready
        buffer.remove2();
        buffer.setReady();

        // handle only one buffer
        break;
    }

    // check if there is a pending bulk transfer
    if (st.state == State::READY) {
        for (auto &buffer : transfers_) {
            int endpointIndex = buffer.endpoint_.index_;

            std::cout << endpointIndex << ": ";
            if (text_) {
                // text
                std::cout << buffer.string();
            } else {
                // binary
                int transferred = buffer.size_;
                std::cout << '(' << transferred << ") ";
                for (int i = 0; i < transferred; ++i) {
                    if ((i & 15) == 0) {
                        if (i != 0)
                            std::cout << "," << std::endl;
                    } else {
                        std::cout << ", ";
                    }
                    std::cout << "0x" << std::setw(2) << std::setfill('0') << std::hex << int(buffer.data()[i]);
                }
                std::cout << std::endl;
            }

            // remove buffer from transfers and set ready
            buffer.remove2();
            buffer.setReady();

            // handle only one buffer
            break;
        }
    }

    // invoke handle() again when there are more pending transfers
    if (!controlTransfers_.empty() || (!transfers_.empty() && st.state == State::READY))
        loop_.invoke(callback_);
}


// ControlBuffer

UsbDevice_cout::ControlBuffer::ControlBuffer(int capacity, UsbDevice_cout &device)
    : coco::Buffer(new uint8_t[capacity], capacity, device.st.state)
    , device_(device)
{
    device.controlBuffers_.add(*this);
}

UsbDevice_cout::ControlBuffer::~ControlBuffer() {
    delete [] data_;
}

bool UsbDevice_cout::ControlBuffer::start(Op op) {
    if (st.state != State::READY) {
        assert(st.state != State::BUSY);
        return false;
    }

    // check if READ or WRITE flag is set
    assert((op & Op::READ_WRITE) != 0);

    //xferred = size;

    auto &device = device_;

    device.controlTransfers_.add(*this);
    device.loop_.invoke(device.callback_);

    // set state
    setBusy();

    return true;
}

bool UsbDevice_cout::ControlBuffer::cancel() {
    if (st.state != State::BUSY)
        return false;

    remove2();

    // cancel takes effect immediately
    setReady(0);
    return true;
}


// Buffer

UsbDevice_cout::Buffer::Buffer(int capacity, Endpoint &endpoint)
    : coco::Buffer(new uint8_t[capacity], capacity, endpoint.device_.st.state)
    , endpoint_(endpoint)
{
    endpoint.buffers_.add(*this);
}

UsbDevice_cout::Buffer::~Buffer() {
    delete [] data_;
}

bool UsbDevice_cout::Buffer::start(Op op) {
    if (st.state != State::READY) {
        assert(st.state != State::BUSY);
        return false;
    }

    // check if READ or WRITE flag is set
    assert((op & Op::READ_WRITE) != 0);

    auto &device = endpoint_.device_;
    device.transfers_.add(*this);
    device.loop_.invoke(device.callback_);

    // set state
    setBusy();

    return true;
}

bool UsbDevice_cout::Buffer::cancel() {
    if (st.state != State::BUSY)
        return false;

    remove2();

    // cancel takes effect immediately
    setReady(0);
    return true;
}


// Endpoint

UsbDevice_cout::Endpoint::Endpoint(UsbDevice_cout &device, int index)
    : BufferDevice(State::OPENING), device_(device), index_(index)
{
    device_.endpoints_.add(*this);
}

UsbDevice_cout::Endpoint::~Endpoint() {
}

int UsbDevice_cout::Endpoint::getBufferCount() {
    return buffers_.count();
}

UsbDevice_cout::Buffer &UsbDevice_cout::Endpoint::getBuffer(int index) {
    return buffers_.get(index);
}

} // namespace coco

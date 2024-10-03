#include "UsbDevice_cout.hpp"
#include <iostream>
#include <iomanip>


namespace coco {

UsbDevice_cout::UsbDevice_cout(Loop_native &loop)
    : UsbDevice(State::OPENING)
    , loop(loop), callback(makeCallback<UsbDevice_cout, &UsbDevice_cout::handle>(this))
{
    // call handle() from event loop to get the device descriptor
    loop.invoke(this->callback);
}

//StateTasks<const Device::State, Device::Events> &UsbDevice_cout::getStateTasks() {
//	return makeConst(this->st);
//}

/*UsbDevice::State UsbDevice_cout::state() {
    return this->stat;
}

Awaitable<Device::Condition> UsbDevice_cout::until(Condition condition) {
    // check if IN_* condition is met
    if ((int(condition) >> int(this->stat)) & 1)
        return {}; // don't wait
    return {this->stateTasks, condition};
}*/

usb::Setup UsbDevice_cout::getSetup() {
    uint16_t value = int(usb::DescriptorType::DEVICE) << 8;
    uint16_t index = 0;
    uint16_t length = sizeof(usb::DeviceDescriptor);
    return {usb::RequestType::STANDARD_DEVICE_IN, uint8_t(usb::Request::GET_DESCRIPTOR), value, index, length};
}

void UsbDevice_cout::acknowledge() {
}

void UsbDevice_cout::stall() {
}

void UsbDevice_cout::handle() {
    // on first call to handle(), get device descriptor from user code
    if (this->readDescriptor) {
        this->readDescriptor = false;

        // enable control buffers
        //for (auto &buffer : this->controlBuffers) {
        //	buffer.setReady(0);
        //}

        // resume the application waiting for an event which should then call getSetup() and send the device descriptor using a ControlBuffer
        this->st.doAll(Events::REQUEST);
        //this->stateTasks.doAll([](Condition condition) {
        //    return (condition & Condition::EVENT1) != 0;
        //});
    }

    // check if there is a pending control transfer
    for (auto &buffer : this->controlTransfers) {
        // get device descriptor
        auto &deviceDescriptor = buffer.value<usb::DeviceDescriptor>();
        this->text = deviceDescriptor.bDeviceProtocol == 1;

        // enable bulk buffers
        /*for (auto &endpoint : this->bulkEndpoints) {
            for (auto &buffer : endpoint.buffers) {
                buffer.setReady(0);
            }
        }*/

        // set ready and resume all coroutines waiting for state change
        for (auto &endpoint : this->endpoints)
            endpoint.st.set(State::READY, Events::ENTER_READY);
        this->st.set(State::READY, Events::ENTER_READY);

        buffer.remove2();
        buffer.setReady();

        // handle only one buffer
        break;
    }

    // check if there is a pending bulk transfer
    if (this->st.state == State::READY) {
        for (auto &buffer : this->transfers) {
            int endpointIndex = buffer.endpoint.index;

            std::cout << endpointIndex << ": ";
            if (this->text) {
                // text
                std::cout << buffer.string();
            } else {
                // binary
                int transferred = buffer.p.size;
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

            buffer.remove2();
            buffer.setReady();

            // handle only one buffer
            break;
        }
    }

    // invoke handle() again when there are more pending transfers
    if (!this->controlTransfers.empty() || (!this->transfers.empty() && this->st.state == State::READY))
        this->loop.invoke(this->callback);
}


// ControlBuffer

UsbDevice_cout::ControlBuffer::ControlBuffer(int capacity, UsbDevice_cout &device)
    : coco::Buffer(new uint8_t[capacity], capacity, device.st.state)
    , device(device)
{
    device.controlBuffers.add(*this);
}

UsbDevice_cout::ControlBuffer::~ControlBuffer() {
    delete [] this->p.data;
}

bool UsbDevice_cout::ControlBuffer::start(Op op) {
    if (this->st.state != State::READY) {
        assert(this->st.state != State::BUSY);
        return false;
    }

    // check if READ or WRITE flag is set
    assert((op & Op::READ_WRITE) != 0);

    //this->xferred = size;

    this->device.controlTransfers.add(*this);
    this->device.loop.invoke(this->device.callback);

    // set state
    setBusy();

    return true;
}

bool UsbDevice_cout::ControlBuffer::cancel() {
    if (this->st.state != State::BUSY)
        return false;

    this->remove2();

    // cancel takes effect immediately
    setReady(0);
    return true;
}


// Buffer

UsbDevice_cout::Buffer::Buffer(int capacity, Endpoint &endpoint)
    : coco::Buffer(new uint8_t[capacity], capacity, endpoint.device.st.state)
    , endpoint(endpoint)
{
    endpoint.buffers.add(*this);
}

UsbDevice_cout::Buffer::~Buffer() {
    delete [] this->p.data;
}

bool UsbDevice_cout::Buffer::start(Op op) {
    if (this->st.state != State::READY) {
        assert(this->st.state != State::BUSY);
        return false;
    }

    // check if READ or WRITE flag is set
    assert((op & Op::READ_WRITE) != 0);

    //this->xferred = size;

    auto &device = this->endpoint.device;
    device.transfers.add(*this);
    device.loop.invoke(device.callback);

    // set state
    setBusy();

    return true;
}

bool UsbDevice_cout::Buffer::cancel() {
    if (this->st.state != State::BUSY)
        return false;

    this->remove2();

    // cancel takes effect immediately
    setReady(0);
    return true;
}


// Endpoint

UsbDevice_cout::Endpoint::Endpoint(UsbDevice_cout &device, int index)
    : BufferDevice(State::OPENING), device(device), index(index)
{
    device.endpoints.add(*this);
}

UsbDevice_cout::Endpoint::~Endpoint() {
}

//StateTasks<const Device::State, Device::Events> &UsbDevice_cout::Endpoint::getStateTasks() {
//	return makeConst(this->device.st);
//}

/*Device::State UsbDevice_cout::BulkEndpoint::state() {
    return this->device.stat;
}

Awaitable<Device::Condition> UsbDevice_cout::BulkEndpoint::until(Condition condition) {
    // check if IN_* condition is met
    if ((int(condition) >> int(this->device.stat)) & 1)
        return {}; // don't wait
    return {this->device.stateTasks, condition};
}*/

int UsbDevice_cout::Endpoint::getBufferCount() {
    return this->buffers.count();
}

UsbDevice_cout::Buffer &UsbDevice_cout::Endpoint::getBuffer(int index) {
    return this->buffers.get(index);
}

} // namespace coco

#include "UsbDevice_cout.hpp"
#include <iostream>
#include <iomanip>


namespace coco {

UsbDevice_cout::UsbDevice_cout(Loop_native &loop)
	: loop(loop), callback(makeCallback<UsbDevice_cout, &UsbDevice_cout::handle>(this))
{
	// call handle() from event loop to get the device descriptor
	loop.invoke(this->callback);
}

UsbDevice::State UsbDevice_cout::state() {
	return this->stat;
}

Awaitable<> UsbDevice_cout::stateChange(int waitFlags) {
	if ((waitFlags & (1 << int(this->stat))) == 0)
		return {};
	return {this->stateTasks};
}

Awaitable<usb::Setup *> UsbDevice_cout::request(usb::Setup &setup) {
	return {this->requestTasks, &setup};
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
		for (auto &buffer : this->controlBuffers) {
			buffer.setReady(0);
		}

		// resume first coroutine waiting for a control request which should write the device descriptor using a ControlBuffer
		this->requestTasks.doFirst([](usb::Setup *setup) {
			uint16_t value = int(usb::DescriptorType::DEVICE) << 8;
			uint16_t index = 0;
			uint16_t length = sizeof(usb::DeviceDescriptor);
			*setup = {usb::RequestType::STANDARD_DEVICE_IN, uint8_t(usb::Request::GET_DESCRIPTOR), value, index, length};
			return true;
		});

		// change state and resume all coroutines waiting for ready state
		this->stat = State::READY;
		this->stateTasks.doAll();
	}

	// check if there is a pending control transfer
	for (auto &buffer : this->controlTransfers) {
		// get device descriptor
		auto &deviceDescriptor = buffer.value<usb::DeviceDescriptor>();
		this->text = deviceDescriptor.bDeviceProtocol == 1;

		// enable bulk buffers
		for (auto &endpoint : this->bulkEndpoints) {
			for (auto &buffer : endpoint.buffers) {
				buffer.setReady(0);
			}
		}

		buffer.remove2();
		buffer.setReady(buffer.xferred);

		// handle only one buffer
		break;
	}

	// check if there is a pending bulk transfer
	for (auto &buffer : this->bulkTransfers) {
		int endpointIndex = buffer.endpoint.index;
		int transferred = buffer.xferred;

		std::cout << endpointIndex << ": ";
		if (this->text) {
			// text
			std::cout << std::string(buffer.array<char>().data(), transferred);
		} else {
			// binary
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
		buffer.setReady(transferred);

		// handle only one buffer
		break;
	}

	if (!this->controlTransfers.empty() || !this->bulkTransfers.empty())
		this->loop.invoke(this->callback);
}


// ControlBuffer

UsbDevice_cout::ControlBuffer::ControlBuffer(UsbDevice_cout &device, int size)
	: BufferImpl(new uint8_t[size], size, device.stat)
	, device(device)
{
	device.controlBuffers.add(*this);
}

UsbDevice_cout::ControlBuffer::~ControlBuffer() {
	delete [] this->dat;
}

bool UsbDevice_cout::ControlBuffer::startInternal(int size, Op op) {
	if (this->stat != State::READY) {
		assert(false);
		return false;
	}

	// check if READ or WRITE flag is set
	assert((op & Op::READ_WRITE) != 0);

	this->xferred = size;

	this->device.controlTransfers.add(*this);
	this->device.loop.invoke(this->device.callback);

	// set state
	setBusy();

	return true;
}

void UsbDevice_cout::ControlBuffer::cancel() {
	if (this->stat != State::BUSY)
		return;

	this->remove2();

	// cancel takes effect immediately
	setReady(0);
}


// BulkBuffer

UsbDevice_cout::BulkBuffer::BulkBuffer(BulkEndpoint &endpoint, int size)
	: BufferImpl(new uint8_t[size], size, endpoint.device.stat)
	, endpoint(endpoint)
{
	endpoint.buffers.add(*this);
}

UsbDevice_cout::BulkBuffer::~BulkBuffer() {
	delete [] this->dat;
}

bool UsbDevice_cout::BulkBuffer::startInternal(int size, Op op) {
	if (this->stat != State::READY) {
		assert(this->stat != State::BUSY);
		return false;
	}

	// check if READ or WRITE flag is set
	assert((op & Op::READ_WRITE) != 0);

	this->xferred = size;

	auto &device = this->endpoint.device;
	device.bulkTransfers.add(*this);
	device.loop.invoke(device.callback);

	// set state
	setBusy();

	return true;
}

void UsbDevice_cout::BulkBuffer::cancel() {
	if (this->stat != State::BUSY)
		return;

	this->remove2();

	// cancel takes effect immediately
	setReady(0);
}


// BulkEndpoint

UsbDevice_cout::BulkEndpoint::BulkEndpoint(UsbDevice_cout &device, int index)
	: device(device), index(index)
{
	device.bulkEndpoints.add(*this);
}

UsbDevice_cout::BulkEndpoint::~BulkEndpoint() {
}

Device::State UsbDevice_cout::BulkEndpoint::state() {
	return this->device.stat;
}

Awaitable<> UsbDevice_cout::BulkEndpoint::stateChange(int waitFlags) {
	if ((waitFlags & (1 << int(this->device.stat))) == 0)
		return {};
	return {this->device.stateTasks};
}

int UsbDevice_cout::BulkEndpoint::getBufferCount() {
	return this->buffers.count();
}

UsbDevice_cout::BulkBuffer &UsbDevice_cout::BulkEndpoint::getBuffer(int index) {
	return this->buffers.get(index);
}

} // namespace coco

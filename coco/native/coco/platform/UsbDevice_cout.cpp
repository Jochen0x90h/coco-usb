#include "UsbDevice_cout.hpp"
#include <iostream>
#include <iomanip>


namespace coco {

UsbDevice_cout::UsbDevice_cout(Loop_native &loop)
	: loop(loop)
{
	// call onSetConfiguration from event loop
	loop.yieldHandlers.add(*this);
}

UsbDevice::State UsbDevice_cout::getState() {
	return this->state;
}

Awaitable<UsbDevice::State> UsbDevice_cout::targetState(State state) {
	if (this->state == state)
		return {};
	return {this->stateWaitlist, state};
}

Awaitable<usb::Setup *> UsbDevice_cout::request(usb::Setup &setup) {
	return {this->requestWaitlist, &setup};
}

Awaitable<UsbDevice::ControlParameters> UsbDevice_cout::controlTransfer(void *data, int size) {
	auto &deviceDescriptor = *reinterpret_cast<usb::DeviceDescriptor *>(data);
	this->text = deviceDescriptor.bDeviceProtocol == 1;
	return {};
}

void UsbDevice_cout::acknowledge() {
}

void UsbDevice_cout::stall() {
}

void UsbDevice_cout::handle() {
	this->remove();

	// get device descriptor from user code
	if (this->readDescriptor) {
		this->readDescriptor = false;

		// resume first coroutine waiting for a control request which should call controlTransfer()
		this->requestWaitlist.resumeFirst([](usb::Setup *setup) {
			uint16_t value = int(usb::DescriptorType::DEVICE) << 8;
			uint16_t index = 0;
			uint16_t length = sizeof(usb::DeviceDescriptor);
			*setup = {usb::RequestType::STANDARD_DEVICE_IN, uint8_t(usb::Request::GET_DESCRIPTOR), value, index, length};
			return true;
		});

		// change state and resume all coroutines waiting for connected state
		this->state = State::CONNECTED;
		this->stateWaitlist.resumeAll([](State state) {
			return state == State::CONNECTED;
		});
	}

	// iterate over all endpoints
	this->writeWaitlist.resumeFirst([this](Stream::WriteParameters &p) {
		auto endpoint = reinterpret_cast<BulkEndpoint *>(p.context);
		int endpointIndex = endpoint->index;
		std::cout << endpointIndex << ": ";
		if (this->text) {
			std::cout << std::string(reinterpret_cast<char const *>(p.data), p.size);
		} else {
			// binary
			std::cout << '(' << p.size << ") ";
			for (int i = 0; i < p.size; ++i) {
				if ((i & 15) == 0) {
					if (i != 0)
						std::cout << "," << std::endl;
				} else {
					std::cout << ", ";
				}
				std::cout << "0x" << std::setw(2) << std::setfill('0') << std::hex << int(reinterpret_cast<uint8_t const *>(p.data)[i]);
			}
			std::cout << std::endl;
		}
		return true;
	});
}


// BulkEndpoint

UsbDevice_cout::BulkEndpoint::~BulkEndpoint() {	
}

static void cancelRead(Stream::ReadParameters &p) {
	p.remove();
}

Awaitable<Stream::ReadParameters> UsbDevice_cout::BulkEndpoint::read(void *data, int &size) {
	if (!this->device.inList()) {
		this->device.loop.yieldHandlers.add(this->device);
	}
	return {this->device.readWaitlist, data, &size, this, cancelRead};
}

static void cancelWrite(Stream::WriteParameters &p) {
	p.remove();
}

Awaitable<Stream::WriteParameters> UsbDevice_cout::BulkEndpoint::write(void const *data, int size) {
	if (!this->device.inList()) {
		this->device.loop.yieldHandlers.add(this->device);
	}
	return {this->device.writeWaitlist, data, size, this, cancelWrite};
}

} // namespace coco

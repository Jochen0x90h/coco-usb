#pragma once

#include "usb.hpp"
#include <coco/Coroutine.hpp>


namespace coco {

/**
 * Interface for an USB device from the host side
 * 
 * https://www.beyondlogic.org/usbnutshell/usb1.shtml
 */
class UsbHostDevice {
public:

	enum class State {
		DISCONNECTED,
		CONNECTED
	};

	struct ControlParameters : public WaitlistElement {
		usb::RequestType requestType;
		uint8_t request;
		uint16_t value;
		uint16_t index;
		void *data;
		uint16_t length;
		void *context;
		void (*cancelCallback)(ControlParameters &);

		// default constructor
		ControlParameters() = default;

		// constructor
		ControlParameters(usb::RequestType requestType, uint8_t request, uint16_t value, uint16_t index, void *data,
			uint16_t length, void *context, void (*cancelCallback)(ControlParameters &)) 
			: requestType(requestType), request(request), value(value), index(index), data(data), length(length), context(context), cancelCallback(cancelCallback) {}
			
		// cancel read operation
		void cancel() {this->cancelCallback(*this);}
	};


	virtual ~UsbHostDevice();

	/**
	 * Get the current state of the device 
	 */
	virtual State getState() = 0;
	bool isConnected() {return getState() == State::CONNECTED;}

	/**
	 * Wait until the device is in the given target state (e.g. co_await device.targetState(State::CONNECTED))
	 * @param state target state
	 * @return use co_await on return value to await the given state
	 */
	[[nodiscard]] virtual Awaitable<State> targetState(State state) = 0;
/*
	virtual void getDescriptor(usb::DescriptorType type, void *data, int &size) = 0;

	template <int N>
	void getDescriptor(DeviceDescriptorBuffer<N> &buffer) {
		buffer.length = N;
		getDescriptor(usb::DescriptorType::DEVICE, buffer.data, buffer.length);
	}
	template <int N>
	void getDescriptor(ConfigurationDescriptorBuffer<N> &buffer) {
		buffer.length = N;
		getDescriptor(usb::DescriptorType::CONFIGURATION, buffer.data, buffer.length);
	}
*/
	/**
	 * Control transfer to/from the device
	 * @param requestType request type including the direction (IN: from device, OUT: to device)
	 */
	[[nodiscard]] virtual Awaitable<ControlParameters> controlTransfer(usb::RequestType requestType, uint8_t request,
		uint16_t value, uint16_t index, void *data, uint16_t length) = 0;


	//virtual void setInterface(int index) = 0;
};

} // namespace coco

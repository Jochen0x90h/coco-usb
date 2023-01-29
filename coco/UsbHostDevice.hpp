#pragma once

#include "usb.hpp"
#include <coco/Buffer.hpp>
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

	// Internal helper: Stores the parameters in Awaitable<> during co_await
	struct ControlParameters : public WaitlistElement {
		usb::Setup setup;
		void *data;
		int size;
		void *context;
		void (*cancelCallback)(ControlParameters &);

		// default constructor
		ControlParameters() = default;

		// constructor
		ControlParameters(usb::Setup const &setup, void *data, int size,
			void *context, void (*cancelCallback)(ControlParameters &)) 
			: setup(setup), data(data), size(size), context(context), cancelCallback(cancelCallback) {}
			
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
	 * @param setup control request setup packet where setup.requestType defines the data direction (IN: from device, OUT: to device)
	 * @param data data to transfer
	 * @param size size of data to transfer
	 * @return use co_await on return value to await completion
	 */
	[[nodiscard]] virtual Awaitable<ControlParameters> controlTransfer(const usb::Setup &setup,
		void *data, int size) = 0;

	template <typename T, int N>
	[[nodiscard]] Awaitable<ControlParameters> controlIn(const usb::Setup &setup, Buffer<T, N> &buffer) {
		buffer.length = N * sizeof(T);
		return read(buffer.buffer, buffer.length);
	}

	template <typename T>
	[[nodiscard]] Awaitable<ControlParameters> controlOut(const usb::Setup &setup, const T &array) {
		const void *data = std::data(array);
		return write(const_cast<void *>(data), std::size(array) * sizeof(*std::data(array)));
	}


	//virtual void setInterface(int index) = 0;
};

} // namespace coco

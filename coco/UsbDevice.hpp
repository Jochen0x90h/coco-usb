#pragma once

#include "usb.hpp"
#include <coco/ArrayConcept.hpp>
#include <coco/Buffer.hpp>
#include <coco/Coroutine.hpp>
#include <coco/PointerConcept.hpp>


namespace coco {

/**
 * Interface for USB device support
 * 
 * https://www.beyondlogic.org/usbnutshell/usb1.shtml
 */
class UsbDevice {
public:

	enum class State {
		DISCONNECTED,
		CONNECTED
	};

	// Internal helper: Stores the parameters in Awaitable<> during co_await
	struct ControlParameters : public WaitlistElement {
		void *data;
		int size;
		void *context;
		void (*cancelCallback)(ControlParameters &);

		// default constructor
		ControlParameters() = default;

		// constructor
		ControlParameters(void *data, int size, void *context, void (*cancelCallback)(ControlParameters &))
			: data(data), size(size), context(context), cancelCallback(cancelCallback) {}
			
		// cancel read operation
		void cancel() {this->cancelCallback(*this);}
	};


	virtual ~UsbDevice();

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

	/**
	 * Wait for a request from the host
	 * @param setup contents of the setup packet where setup.requestType determines the direction
	 * @return use co_await on return value to wait for a request
	 */
	[[nodiscard]] virtual Awaitable<usb::Setup *> request(usb::Setup &setup) = 0;

	/**
	 * Control transfer to/from the device
	 * @param data data to transfer
	 * @param size size of data to transfer
	 * @return use co_await on return value to await completion
	 */
	[[nodiscard]] virtual Awaitable<ControlParameters> controlTransfer(void *data, int size) = 0;

	template <typename T> requires (ArrayConcept<T>)
	[[nodiscard]] Awaitable<ControlParameters> controlIn(usb::Setup const &setup, const T &array) {
		const void *d = std::data(array);
		int size = std::min(int(setup.length), std::size(array) * sizeof(*std::data(array)));
		return controlTransfer(const_cast<void *>(d), size);
	}

	template <typename T> requires (PointerConcept<T>)
	[[nodiscard]] Awaitable<ControlParameters> controlIn(usb::Setup const &setup, T pointer) {
		auto &data = *pointer;
		int size = std::min(int(setup.length), int(sizeof(data)));
		return controlTransfer((void *)pointer, size);
	}

	template <typename T>
	[[nodiscard]] Awaitable<ControlParameters> controlOut(usb::Setup const &setup, T *data) {
		int size = std::min(int(setup.length), int(sizeof(T)));
		return controlTransfer(data, size);
	}

	template <typename T, int N>
	[[nodiscard]] Awaitable<ControlParameters> controlOut(usb::Setup const &setup, Buffer<T, N> &buffer) {
		int size = std::min(int(setup.length), N);
		buffer.length = size;
		return controlTransfer(buffer.buffer, size);
	}

	/**
	 * Acknowledge a conrol request without data stage
	 */
	virtual void acknowledge() = 0;

	/**
	 * Indicate that a control request is not supported or has invalid parameters
	 */
	virtual void stall() = 0;
};

} // namespace coco

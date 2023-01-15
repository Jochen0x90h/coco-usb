#pragma once

#include "usb.hpp"
#include <coco/Coroutine.hpp>
//#include <Data.hpp>
#include <functional>


namespace coco {

/**
 * Data wrapper, only references the data
 */
class ConstData {
public:

	constexpr ConstData() : s(0), d(nullptr) {}

	ConstData(void const *data, int size) : s(size), d(data) {}

	template <typename T>
	constexpr ConstData(T const *data) : s(sizeof(T)), d(data) {}

	/**
	 * Get size of data in bytes
	 * @return size
	 */
	int size() {return this->s;}

	/**
	 * Get pointer to the data
	 * @return data
	 */
	void const *data() {return this->d;}

	template <typename T>
	T const *cast() {
		assert(sizeof(T) >= this->s);
		return reinterpret_cast<T const *>(this->d);
	}

protected:

	int const s;
	void const *const d;
};



/**
 * Interface for USB device support
 * 
 * https://www.beyondlogic.org/usbnutshell/usb1.shtml
 */
class UsbDevice {
public:

	// Internal helper: Stores the parameters and a reference to the result value in the awaitable during co_await
	struct ReceiveParameters {
		void *data;
		int &size;
	};

	// Internal helper: Stores the parameters in the awaitable during co_await
	struct SendParameters {
		void const *data;
		int size;
	};


	virtual ~UsbDevice();

	/**
	 * Enable endpoints. Can be done in onSetConfiguration. Endpoint 0 should stay enabled
	 * @param inFlags an enabled flag for each in endpoint
	 * @param outFlags an enabled flag for each out endpoint
	 */
	virtual void enableEndpoints(int inFlags, int outFlags) = 0;

	/**
	 * Receive data from the host via an endpoint (OUT transfer)
	 * @param index endpoint index
	 * @param data data to receive, must be in RAM dependent on driver
	 * @param size in: size of data buffer, out: number of bytes actually received
	 * @return use co_await on return value to await completion
	 */
	[[nodiscard]] virtual Awaitable<ReceiveParameters> receive(int index, void *data, int &size) = 0;

	/**
	 * Receive data from the host via an endpoint (OUT transfer)
	 * @param index endpoint index
	 * @param data data to receive, must be in RAM dependent on driver
	 * @param size size of data buffer
	 * @param transferred number of bytes actually received
	 * @return use co_await on return value to await completion
	 */
	[[nodiscard]] Awaitable<ReceiveParameters> receive(int index, void *data, int size, int &transferred) {
		transferred = size;
		return receive(index, data, transferred);
	}

	/**
	 * Send data to the host via an endpoint (IN transfer)
	 * @param index endpoint index
	 * @param data data to send, must be in RAM dependent on driver
	 * @param size size of data buffer
	 * @return use co_await on return value to await completion
	 */
	[[nodiscard]] virtual Awaitable<SendParameters> send(int index, void const *data, int size) = 0;

};

} // namespace coco

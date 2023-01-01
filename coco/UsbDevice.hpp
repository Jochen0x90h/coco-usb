#pragma once

#include "usb.hpp"
#include <coco/Coroutine.hpp>
//#include <Data.hpp>
#include <cstdint>
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
 * Interface to an USB device
 * 
 * https://www.beyondlogic.org/usbnutshell/usb1.shtml
 */
class UsbDevice {
public:

	// Internal helper: Stores the parameters and a reference to the result value in the awaitable during co_await
	struct ReceiveParameters {
		void *data;
		int &length;
	};

	// Internal helper: Stores the parameters in the awaitable during co_await
	struct SendParameters {
		void const *data;
		int length;
	};


	virtual ~UsbDevice();

	/**
	 * Enable endpoints. Can be done in onSetConfiguration. Endpoint 0 should stay enabled
	 * @param inFlags an enabled flag for each in endpoint
	 * @param outFlags an enabled flag for each out endpoint
	 */
	virtual void enableEndpoints(uint8_t inFlags, uint8_t outFlags) = 0;

	/**
	 * Suspend execution using co_await until data is received from an endpoint (OUT transfer)
	 * @param index endpoint index (1-7)
	 * @param length in: length of data buffer, out: length of data actually received
	 * @param data data to receive, must be in RAM
	 */
	[[nodiscard]] virtual Awaitable<ReceiveParameters> receive(int index, void *data, int &length) = 0;
	[[nodiscard]] Awaitable<ReceiveParameters> receive(int index, void *data, int length, int &transferred) {
		transferred = length;
		return receive(index, data, transferred);
	}

	/**
	 * Suspend execution using co_await until data is sent over an endpoint (IN transfer)
	 * @param index endpoint index (1-7)
	 * @param length data length
	 * @param data data to send, must be in RAM
	 */
	[[nodiscard]] virtual Awaitable<SendParameters> send(int index, void const *data, int length) = 0;

};

} // namespace coco

#pragma once

#include <coco/Coroutine.hpp>
#include <coco/Buffer.hpp>
#include <iterator>


namespace coco {

class InputStream {
public:

	struct ReadParameters : public WaitlistElement {
		void *data;
		int *size;
		void *context;
		void (*cancelCallback)(ReadParameters &);

		// default constructor
		ReadParameters() = default;

		// constructor
		ReadParameters(void *data, int *size, void *context, void (*cancelCallback)(ReadParameters &)) 
			: data(data), size(size), context(context), cancelCallback(cancelCallback) {}
			
		// cancel read operation
		void cancel() {this->cancelCallback(*this);}
	};

	virtual ~InputStream();

	/**
	 * Read data from the stream
	 * @param data data to receive, must be in RAM dependent on driver
	 * @param size in: size of data buffer, out: number of bytes actually received
	 * @return use co_await on return value to await completion
	 */
	[[nodiscard]] virtual Awaitable<ReadParameters> read(void *data, int &size) = 0;

	/**
	 * Read data from the stream
	 * @param data data to receive, must be in RAM dependent on driver
	 * @param size size of data buffer
	 * @param transferred number of bytes actually received
	 * @return use co_await on return value to await completion
	 */
	[[nodiscard]] Awaitable<ReadParameters> read(void *data, int size, int &transferred) {
		transferred = size;
		return read(data, transferred);
	}

	template <typename T, int N>
	[[nodiscard]] Awaitable<ReadParameters> read(Buffer<T, N> &buffer) {
		buffer.length = N * sizeof(T);
		return read(buffer.buffer, buffer.length);
	}
};

class OutputStream {
public:

	struct WriteParameters : public WaitlistElement {
		const void *data;
		int size;
		void *context;
		void (*cancelCallback)(WriteParameters &);

		// default constructor
		WriteParameters() = default;

		// constructor
		WriteParameters(const void *data, int size, void *context, void (*cancelCallback)(WriteParameters &)) 
			: data(data), size(size), context(context), cancelCallback(cancelCallback) {}
			
		// cancel write operation
		void cancel() {this->cancelCallback(*this);}
	};

	virtual ~OutputStream();
	
	/**
	 * Write data to the stream
	 * @param data data to send, must be in RAM dependent on driver
	 * @param size size of data buffer
	 * @return use co_await on return value to await completion
	 */
	[[nodiscard]] virtual Awaitable<WriteParameters> write(const void *data, int size) = 0;

	template <typename T> requires (!CStringConcept<T>)
	[[nodiscard]] Awaitable<WriteParameters> write(const T &array) {
		return write(std::data(array), std::size(array) * sizeof(*std::data(array)));
	}

	template <typename T> requires (CStringConcept<T>)
	[[nodiscard]] Awaitable<WriteParameters> write(const T &string) {
		String str(string);
		return write(str.data(), str.size());
	}
};

class Stream : public InputStream, public OutputStream {
public:

	virtual ~Stream();

};

} // namespace coco

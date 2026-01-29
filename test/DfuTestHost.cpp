#include <coco/Coroutine.hpp>
#include <coco/String.hpp>
#include <coco/StringBuffer.hpp>
#include <coco/StreamOperators.hpp>
#include <DfuTestHost.hpp>
#include <iostream>
#include <iomanip>


// Host for testing DFU devices


using namespace coco;


// vendor specific control request
enum class Request : uint8_t {
    RED = 0,
    GREEN = 1,
    BLUE = 2,
    COLOR = 3,
    DEVICE_ID = 4,
    VARIANT_ID = 5
};

inline auto controlOut(Buffer &buffer, Request request, uint16_t wValue, uint16_t wIndex) {
    buffer.header<usb::Setup>() = {usb::RequestType::VENDOR_DEVICE_OUT, uint8_t(request), wValue, wIndex, 0};
    return buffer.write(0);
}

inline auto controlOut(Buffer &buffer, Request request, uint16_t wValue, uint16_t wIndex, int32_t data) {
    buffer.header<usb::Setup>() = {usb::RequestType::VENDOR_DEVICE_OUT, uint8_t(request), wValue, wIndex, 4};
    return buffer.writeValue(data);
}


// print test status
void printLength(int endpoint, int length) {
    std::cout << endpoint << ": length: " << dec(length) << std::endl;
}

void printStatus(int endpoint, String message, bool ok) {
    std::cout << endpoint << ": " << message << ": ";
    std::cout << (ok ? "ok" : "error!");
    std::cout << std::endl;
}

// test echo coroutine in device (i.e. write to device and check if it returns the same data)
Coroutine dfuTest(Loop &loop, Device &device, Buffer &control) {
    while (true) {
        // wait until USB device gets detected (control buffer becomes ready)
        std::cout << "Wait for USB device..." << std::endl;
        co_await device.untilReady();

        std::cout << "Device is ready" << std::endl;

        // todo

        loop.exit();
        co_return;
    }
}

// test write coroutine in device (i.e. read from device)
Coroutine readTest(Loop &loop, Device &device, Buffer &buffer) {
    while (true) {
        // wait until USB device gets detected (buffer becomes ready)
        std::cout << "Wait for USB device..." << std::endl;
        co_await device.untilReady();

        while (buffer.ready()) {
            std::cout << "wait for read" << std::endl;
            co_await buffer.read();

            std::cout << std::dec << buffer.size() << ' ' << std::hex << int(buffer[0]) << ' ' << int(buffer[1]) << ' ' << int(buffer[2]) << ' ' << int(buffer[8]) << std::endl;
        }
    }
}

int main() {
    Drivers drivers;

    dfuTest(drivers.loop, drivers.device, drivers.controlBuffer);

    drivers.loop.run();

    return 0;
}

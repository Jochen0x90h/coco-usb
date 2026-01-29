#include <coco/Coroutine.hpp>
#include <coco/String.hpp>
#include <coco/StringBuffer.hpp>
#include <coco/StreamOperators.hpp>
#include <UsbTestHost.hpp>
#include <iostream>
#include <iomanip>


// Host for UsbDeviceTest
// Flash UsbDeviceTest onto the device, then run this


using namespace coco;


/*struct dec {
    dec(int i) : i(i) {}
    int i;
};
std::ostream &operator <<(std::ostream &s, Dec v) {
    return s << std::dec << v.i;
}

struct hex {
    hex(uint8_t v) : w(2), v(v) {}
    hex(uint16_t v) : w(4), v(v) {}
    int w;
    unsigned int v;
};
std::ostream &operator <<(std::ostream &s, Hex h) {
    return s << std::setfill('0') << std::setw(h.w) << std::hex << h.v;
}*/


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
Coroutine echoTest(Loop &loop, Device &device, Buffer &control, Buffer &buffer, int endpoint) {
    while (true) {
        // wait until USB device gets detected (control buffer becomes ready)
        std::cout << "Wait for USB device..." << std::endl;
        co_await device.untilReady();

        // test control request
        if (endpoint == 1) {
            for (int i = 1; i <= 8; ++i)
                co_await controlOut(control, Request::COLOR, 0, 256, i);
            co_await controlOut(control, Request::RED, 1, 0); // set on if wValue != 0
            co_await controlOut(control, Request::RED, 0, 0);
            co_await controlOut(control, Request::GREEN, 0, 1); // set on if wIndex != 0
            co_await controlOut(control, Request::GREEN, 0, 0);
            co_await controlOut(control, Request::BLUE, 5, 5); // set on if wValue == wIndex
            co_await controlOut(control, Request::BLUE, 0, 256);
        }

        // flush out data from last run
        for (int i = 0; i < 4; ++i) {
            int r = co_await select(buffer.read(), loop.sleep(100ms));
            std::cout << i << ' ' << r << std::endl;
            if (r == 2) {
                // timeout: cancel read and wait until ready (or disabled)
                co_await buffer.acquire();
                break;
            }
        }

        // echo loop: send data to device and check if we get back the same data
        int sendLength = 64;//128;
        bool allOk = true;
        while (buffer.ready()) {
            // fill buffer with test data
            for (int i = 0; i < sendLength; ++i) {
                buffer[i] = sendLength + i;
            }
            std::cout << endpoint << ": send length: " << sendLength << std::endl;

            // send to device and receive reply (we get back the same data that we sent)
            //co_await buffer.write(sendLength);
            //co_await buffer.read();
            co_await buffer.write(sendLength, Buffer::Op::READ);

            int transferred = buffer.size();
            printStatus(endpoint, "receive", transferred == sendLength);
            allOk &= transferred == sendLength;

            // check received data
            bool ok = true;
            for (int i = 0; i < sendLength; ++i) {
                if (buffer[i] != uint8_t(sendLength + i))
                    ok = false;
            }
            printStatus(endpoint, "data", ok);
            allOk &= ok;

            printStatus(endpoint, "all", allOk);

            // wait
            co_await loop.sleep(100ms);

            // modify the send length
            sendLength = (sendLength + 5) % 129;
        }
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

    echoTest(drivers.loop, drivers.device, drivers.controlBuffer, drivers.buffer1, 1);
    echoTest(drivers.loop, drivers.device, drivers.controlBuffer, drivers.buffer2, 2);

    //readTest(drivers.loop, drivers.device, drivers.buffer1);

    drivers.loop.run();

    return 0;
}

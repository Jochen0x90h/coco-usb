#pragma once

#include <coco/platform/Loop_native.hpp>
#include <coco/platform/UsbDevice_cout.hpp>


using namespace coco;

// "fake" USB device that just prints to cout

constexpr int CONTROL_BUFFER_SIZE = 256;

// drivers for UsbTest
struct Drivers {
    Loop_native loop;

    // "fake" USB device
    using Usb = UsbDevice_cout;
    Usb usb{loop};
    Usb::ControlBuffer controlBuffer{CONTROL_BUFFER_SIZE, usb};
    Usb::Endpoint endpoint1{usb, 1};
    Usb::Endpoint endpoint2{usb, 2};
    Usb::Buffer buffer1{129, endpoint1};
    Usb::Buffer buffer2{129, endpoint2};
};

Drivers drivers;

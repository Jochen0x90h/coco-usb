#pragma once

#include <coco/usb.hpp>
#include <coco/platform/Loop_native.hpp>
#include <coco/platform/UsbHost_native.hpp>
#include <coco/platform/UsbMonitor_native.hpp>


using namespace coco;

// drivers for UsbTestHost
struct Drivers {
    Loop_native loop;

    UsbMonitor_native monitor{loop};

    using Usb = UsbHost_native;
    Usb host{loop};
    Usb::Device device{host};
    Usb::ControlBuffer controlBuffer{32, device};
    /*Usb::Endpoint endpoint1{device, 1};
    Usb::Endpoint endpoint2{device, 2};
    Usb::Buffer buffer1{129, endpoint1};
    Usb::Buffer buffer2{129, endpoint2};*/
};

Drivers drivers;

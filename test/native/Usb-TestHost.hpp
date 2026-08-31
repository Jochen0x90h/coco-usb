#pragma once

#include <coco/usb.hpp>
#include <coco/platform/Loop_native.hpp>
#include <coco/platform/UsbHostDevice_native.hpp>
#include <coco/platform/UsbMonitor_native.hpp>


using namespace coco;

// drivers for UsbTestHost
struct Drivers {
    Loop_native loop;

    UsbMonitor_native monitor{loop};

    using UsbDevice = UsbHostDevice_native;
    UsbDevice device{loop};
    UsbDevice::ControlBuffer controlBuffer{32, device};
    UsbDevice::Endpoint endpoint1{device, 1};
    UsbDevice::Endpoint endpoint2{device, 2};
    UsbDevice::Buffer buffer1{129, endpoint1};
    UsbDevice::Buffer buffer2{129, endpoint2};
};

Drivers drivers;

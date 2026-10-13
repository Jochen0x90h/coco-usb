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
};

Drivers drivers;

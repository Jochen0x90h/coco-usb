#pragma once

#include <coco/platform/Loop_native.hpp>
#include <coco/platform/UsbMonitor_native.hpp>


using namespace coco;

// drivers for UsbTestHost
struct Drivers {
    Loop_native loop{true}; // also process window messages

    UsbMonitor_native monitor{loop};
};

Drivers drivers;

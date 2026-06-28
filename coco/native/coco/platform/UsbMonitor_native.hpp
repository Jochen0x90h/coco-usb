#pragma once

#ifdef _WIN32
#include "UsbMonitor_WinUSB.hpp"
namespace coco {
using UsbMonitor_native = UsbMonitor_WinUSB;
}
#endif
#ifdef __linux__
#include "UsbMonitor_udev.hpp"
namespace coco {
using UsbMonitor_native = UsbMonitor_udev;
}
#endif

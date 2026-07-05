#pragma once

#ifdef _WIN32
#include "UsbMonitor_SetupAPI.hpp"
namespace coco {
using UsbMonitor_native = UsbMonitor_SetupAPI;
}
#endif
#ifdef __linux__
#include "UsbMonitor_udev.hpp"
namespace coco {
using UsbMonitor_native = UsbMonitor_udev;
}
#endif

#pragma once

#ifdef _WIN32
#include "UsbHostDevice_WinUSB.hpp"
namespace coco {
using UsbHostDevice_native = UsbHostDevice_WinUSB;
}
#endif
#ifdef __linux__
#include "UsbHostDevice_io_uring.hpp"
namespace coco {
using UsbHostDevice_native = UsbHostDevice_io_uring;
}
#endif

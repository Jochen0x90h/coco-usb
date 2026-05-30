#pragma once

#ifdef _WIN32
#include "UsbHost_WinUSB.hpp"
namespace coco {
using UsbHost_native = UsbHost_WinUSB;
}
#endif
#ifdef __linux__
#include "UsbHost_io_uring.hpp"
namespace coco {
using UsbHost_native = UsbHost_io_uring;
}
#endif

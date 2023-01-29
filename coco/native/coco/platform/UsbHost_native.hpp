#pragma once

#ifdef _WIN32
#include "UsbHost_Win32.hpp"
namespace coco {
using UsbHost_native = UsbHost_Win32;
}
#endif

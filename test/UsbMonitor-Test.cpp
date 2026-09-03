#include <UsbMonitor-Test.hpp>
#include <coco/convert.hpp>
#include <coco/debug.hpp>


// Test USB host device enumeration


using namespace coco;


int main() {
    debug::out << "UsbMonitor-Test\n";

    drivers.monitor.listenAdd([](DevicePath path, auto &descriptor, String manufacturer, String product, String serial) {
        debug::out << hex(descriptor.idVendor) << ':' << hex(descriptor.idProduct) << ' ';
        if (!manufacturer.empty())
            debug::out << manufacturer << ' ';
        if (!product.empty())
            debug::out << product << ' ';
        if (!serial.empty())
            debug::out << '[' << serial << ']';
        debug::out << " (" << path << ")\n";
    });
    drivers.monitor.listenRemove([](DevicePath path) {
        debug::out << " (" << path << ")\n";
    });

    drivers.loop.run();

    return 0;
}

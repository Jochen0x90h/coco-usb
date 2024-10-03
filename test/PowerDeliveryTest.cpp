#include <coco/BufferReader.hpp>
#include <coco/Loop.hpp>
#include <coco/debug.hpp>
#include <PowerDeliveryTest.hpp>



using namespace coco;


Coroutine communicationStateMachine(Loop &loop, UsbPowerDelivery &pd) {
	auto &buffer = pd.getBuffer(0);

	while (true) {
		co_await buffer.read();

		BufferReader r(buffer);

		auto header = r.e16L<usbpd::Hader>();

	}
}

Coroutine connectionStateMachine(Loop &loop, UsbPowerDelivery &pd) {
	while (true) {
		// get connection state
		auto connectionState = pd.getConnectionState();

		// check if connected
		if (connectionState != UsbPowerDelivery::ConnectionState::SINK_OPEN) {
			// debounce
			loop.sleep(100ms);

			// check if same state
			if (pd.getConnectionState() == connectionState) {
				// launch communication state machine
				auto c = communicationStateMachine(loop);

				// wait for state change
				co_await pd.getConnectionStateChanged();

				// kill communication state machine
				c.destroy();

			}
		}
	}
}


int main() {
	connectionStateMachine(drivers.loop, drivers.pd);

	drivers.loop.run();
	return 0;
}

#include <coco/UsbDevice.hpp>
#include <coco/platform/Handler.hpp>


namespace coco {

/**
 * Implementation of an USB device that simply writes info about the transfer operations to std::cout
 */
class UsbDevice_cout : public UsbDevice, public YieldHandler {
public:
	// number of endpoints without endpoint 0
	static constexpr int ENDPOINT_COUNT = 7;

	/**
	 * Constructor
	 * @param getDescriptor callback for obtaining descriptors
	 * @param onSetConfiguration callback for setting the configuration (libusb_set_configuration() on host), always called from event loop
	 * @param onRequest callback for vendor specific request
	 */
	UsbDevice_cout(
		std::function<ConstData (usb::DescriptorType)> const &getDescriptor,
		std::function<void (UsbDevice &usb, uint8_t bConfigurationValue)> const &onSetConfiguration,
		std::function<bool (uint8_t bRequest, uint16_t wValue, uint16_t wIndex)> const &onRequest);

	void enableEndpoints(int inFlags, int outFlags) override;
	[[nodiscard]] Awaitable<ReceiveParameters> receive(int index, void *data, int &size) override;
	[[nodiscard]] Awaitable<SendParameters> send(int index, void const *data, int size) override;

	void activate() override;

protected:

	bool text;
	std::function<void (UsbDevice &, uint8_t)> onSetConfiguration;

	// endpoints 1 - 7
	struct Endpoint {
		Waitlist<ReceiveParameters> receiveWaitlist;
		Waitlist<SendParameters> sendWaitlist;
	};

	Endpoint endpoints[ENDPOINT_COUNT];
};

} // namespace coco

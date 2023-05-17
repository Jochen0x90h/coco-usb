#pragma once

#include <coco/enum.hpp>
#include <cstdint>
#include <iterator>

#ifdef _MSC_VER
#define PACK(Declaration) __pragma(pack(push, 1)) Declaration __pragma(pack(pop))
#else
#define PACK(Declaration) Declaration __attribute__((__packed__))
#endif


namespace coco {
namespace usb {

// descriptor type
enum class DescriptorType : uint8_t {
	DEVICE = 1,
	CONFIGURATION = 2,
	STRING = 3,
	INTERFACE = 4,
	ENDPOINT = 5
};

// endpoint type
enum class EndpointType : uint8_t {
	CONTROL = 0,
	ISOCHRONOUS = 1,
	BULK = 2,
	INTERRUPT = 3
};

// endpoint transfer direction
enum Direction : uint8_t {
	OUT = 0, // to device
	IN = 0x80 // to host
};

// control request type
enum class RequestType : uint8_t {
	// request type
	STANDARD = 0x0 << 5,
	CLASS = 0x1 << 5,
	VENDOR = 0x2 << 5,
	TYPE_MASK = 0x3 << 5,

	// request recipient
	DEVICE = 0x00,
	INTERFACE = 0x01,
	ENDPOINT = 0x02,
	OTHER = 0x03,
	RECIPIENT_MASK = 0x1f,

	// request direction
	OUT = 0, // to device
	IN = 0x80, // to host
	DIRECTION_MASK = 0x80,

	// combinations
	STANDARD_DEVICE_OUT = STANDARD | DEVICE | OUT,
	STANDARD_DEVICE_IN = STANDARD | DEVICE | IN,
	STANDARD_INTERFACE_OUT = STANDARD | INTERFACE | OUT,
	STANDARD_INTERFACE_IN = STANDARD | INTERFACE | IN,
	STANDARD_ENDPOINT_OUT = STANDARD | ENDPOINT | OUT,
	STANDARD_ENDPOINT_IN = STANDARD | ENDPOINT | IN,

	VENDOR_DEVICE_OUT = VENDOR | DEVICE | OUT,
	VENDOR_DEVICE_IN = VENDOR | DEVICE | IN,
	VENDOR_INTERFACE_OUT = VENDOR | INTERFACE | OUT,
	VENDOR_INTERFACE_IN = VENDOR | INTERFACE | IN,
	VENDOR_ENDPOINT_OUT = VENDOR | ENDPOINT | OUT,
	VENDOR_ENDPOINT_IN = VENDOR | ENDPOINT | IN
};
COCO_ENUM(RequestType)

enum Request : uint8_t {
	GET_STATUS = 0x00, // device, interface, endpoint
	CLEAR_FEATURE = 0x01, // device, interface, endpoint
	SET_FEATURE = 0x03, // device, interface, endpoint
	SET_ADDRESS = 0x05, // device
	GET_DESCRIPTOR = 0x06, // device
	SET_DESCRIPTOR = 0x07, // device
	GET_CONFIGURATION = 0x08, // device
	SET_CONFIGURATION = 0x09, // device
	GET_INTERFACE = 0x0a, // interface
	SET_INTERFACE = 0x11, // interface
	SYNCH_FRAME = 0x12 // endpoint
};


// setup packet of control request
struct Setup {
	usb::RequestType requestType;
	uint8_t request;
	uint16_t value;
	uint16_t index;
	uint16_t length;
};

// device descriptor
PACK(struct DeviceDescriptor {
	uint8_t  bLength;
	DescriptorType  bDescriptorType;
	uint16_t bcdUSB;
	uint8_t  bDeviceClass;
	uint8_t  bDeviceSubClass;
	uint8_t  bDeviceProtocol;
	uint8_t  bMaxPacketSize0;
	uint16_t idVendor;
	uint16_t idProduct;
	uint16_t bcdDevice;
	uint8_t  iManufacturer;
	uint8_t  iProduct;
	uint8_t  iSerialNumber;
	uint8_t  bNumConfigurations;
});

// configuration descriptor
PACK(struct ConfigurationDescriptor {
	uint8_t bLength;
	DescriptorType bDescriptorType;
	uint16_t wTotalLength;
	uint8_t bNumInterfaces;
	uint8_t bConfigurationValue; // value to use as an argument to select this configuration
	uint8_t iConfiguration; // index of String Descriptor describing this configuration
	uint8_t bmAttributes;
	uint8_t bMaxPower;
});

// interface descriptor
PACK(struct InterfaceDescriptor {
	uint8_t bLength;
	DescriptorType bDescriptorType;
	uint8_t bInterfaceNumber;
	uint8_t bAlternateSetting;
	uint8_t bNumEndpoints;
	uint8_t bInterfaceClass;
	uint8_t bInterfaceSubClass;
	uint8_t bInterfaceProtocol;
	uint8_t iInterface;
});

// endpoint descriptor
PACK(struct EndpointDescriptor {
	uint8_t bLength;
	DescriptorType bDescriptorType;
	uint8_t bEndpointAddress;
	EndpointType bmAttributes;
	uint16_t wMaxPacketSize;
	uint8_t bInterval;
});

} // namespace usb
} // namespace coco

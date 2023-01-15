#include "UsbHost_Win32.hpp"
#include <coco/convert.hpp>
#include <coco/loop.hpp>
#include <iostream>
#define NOMINMAX
//#include <Windows.h>
#include <SetupAPI.h>
#include <initguid.h>
#include <usbiodef.h>
#include <winusb.h>
#undef IN
#undef OUT
#undef interface


namespace coco {

UsbHost_Win32::UsbHost_Win32() {
	
	// regularly scan for devices
	this->time = loop::now();
	coco::timeHandlers.add(*this);
}

UsbHost_Win32::~UsbHost_Win32() {
}
/*
static int parseHex(const char *s) {
	int value = 0;
	for (int i = 0; i < 4; ++i) {		
		value <<= 4;

		char ch = s[i];
		if (ch >= '0' && ch <= '9')
			value += ch - '0';
		else if (ch >= 'A' && ch <= 'F')
			value += ch - 'A' + 10;
		else if (ch >= 'a' && ch <= 'f')
			value += ch - 'a' + 10;
	}
	return value;
}*/

void UsbHost_Win32::activate() {
	this->time = loop::now() + 1s;
	
	// enumerate devices
	HDEVINFO deviceInfo = SetupDiGetClassDevsA(nullptr, nullptr, nullptr, DIGCF_ALLCLASSES | DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);
	int index = 0;
	SP_DEVINFO_DATA deviceData;
	deviceData.cbSize = sizeof(SP_DEVINFO_DATA);
	while (SetupDiEnumDeviceInfo(deviceInfo, index, &deviceData)) {
		++index;

		// get interface data
		SP_DEVICE_INTERFACE_DATA interfaceData;
		interfaceData.cbSize = sizeof(SP_DEVICE_INTERFACE_DATA);
		if (!SetupDiEnumDeviceInterfaces(deviceInfo, &deviceData, &GUID_DEVINTERFACE_USB_DEVICE, 0, &interfaceData)) {
			continue;
		}
		
		// get device path
		union {
			SP_DEVICE_INTERFACE_DETAIL_DATA_A devicePath;
			uint8_t space[2 + 128];
		} u;
		u.devicePath.cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA_A);
		if (!SetupDiGetDeviceInterfaceDetailA(deviceInfo, &interfaceData, &u.devicePath, sizeof(u), nullptr, nullptr)) {
			// error
			continue;
		}

		// check if device is new
		auto p = this->deviceInfos.insert(std::map<std::string, Device *>::value_type{u.devicePath.DevicePath, nullptr});
		if (p.second) {
			// found a new device, path as the form \\?\usb#vid_1915&pid_1337#5&41045ef&0&4#{a5dcbf10-6530-11d2-901f-00c04fb951ed}
			//auto it = this->deviceInfos.insert(u.devicePath.DevicePath, nullptr).first;

			// try to open the device
			auto file = CreateFileA(p.first->first.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, FILE_FLAG_OVERLAPPED, nullptr);
			if (file == INVALID_HANDLE_VALUE)
				continue;

			// get USB interface
			WINUSB_INTERFACE_HANDLE interface;
			if (!WinUsb_Initialize(file, &interface)) {
				CloseHandle(file);
				continue;
			}

			// read device descriptor
			usb::DeviceDescriptor deviceDescriptor;
		  	ULONG transferred;
			bool result = WinUsb_GetDescriptor(interface, int(usb::DescriptorType::DEVICE), 0, 0, (UCHAR*)&deviceDescriptor, sizeof(deviceDescriptor), &transferred);
			if (!result || transferred < sizeof(deviceDescriptor)) {
				WinUsb_Free(interface);
				CloseHandle(file);
				continue;
			}

			// now check if a filter of a device accepts the device descriptor
			for (Device &device : this->devices) {
				if (device.state == Device::State::DISCONNECTED && device.filter(deviceDescriptor)) {
					// transfer ownership to device
					device.it = p.first;
					p.first->second = &device;
					device.flag = true;
					device.file = file;
					device.interface = interface;
					file = INVALID_HANDLE_VALUE;

					// change state
					device.state = Device::State::CONNECTED;
					
					// resume all coroutines waiting for connected state
					device.stateWaitlist.resumeAll([](Device::State state) {
						return state == Device::State::CONNECTED;
					});
					break;
				}
			}

			if (file != INVALID_HANDLE_VALUE) {
				WinUsb_Free(interface);
				CloseHandle(file);
			}
		} else {
			// flag the device
			if (p.first->second != nullptr)
				p.first->second->flag = true;
		}
	}

	for (Device &device : this->devices) {
		if (device.isConnected() && !device.flag)
			device.disconnect();
		device.flag = false;
	}
}


// Device

UsbHost_Win32::Device::Device(UsbHost_Win32 &host, std::function<bool (const usb::DeviceDescriptor &)> filter)
	: host(host), filter(filter)
{
	host.devices.add(*this);
	coco::yieldHandlers.add(*this);
}

UsbHost_Win32::Device::~Device() {
	WinUsb_Free(this->interface);
	CloseHandle(this->file);
}

UsbHostDevice::State UsbHost_Win32::Device::getState() {
	return this->state;
}

Awaitable<UsbHostDevice::State> UsbHost_Win32::Device::targetState(Device::State state) {
	if (this->state == state)
		return {};
	return {this->stateWaitlist, state};
}

static void cancelControlTransfer(UsbHostDevice::ControlParameters &p) {
	UsbHost_Win32::Device &device = *reinterpret_cast<UsbHost_Win32::Device *>(p.context);

	CancelIoEx(device.file, &device.controlOverlapped);
	
	// wait until cancellation completes
	DWORD transferred;
	GetOverlappedResult(device.file, &device.controlOverlapped, &transferred, true);

	device.transferring = false;
	p.remove();
}

Awaitable<UsbHostDevice::ControlParameters> UsbHost_Win32::Device::controlTransfer(usb::RequestType requestType, uint8_t request,
	uint16_t value, uint16_t index, void *data, uint16_t length)
{
	if (!isConnected())
		return {};
	if (!this->transferring) {
		this->transferring = true;
		WINUSB_SETUP_PACKET packet;
		packet.RequestType = UCHAR(requestType);
		packet.Request = request;
		packet.Value = value;
		packet.Index = index;
		packet.Length = length;
		memset(&this->controlOverlapped, 0, sizeof(OVERLAPPED));
		WinUsb_ControlTransfer(this->interface, packet, (PUCHAR)data, length, nullptr, &controlOverlapped);
	}
	return {this->controlWaitlist, requestType, request, value, index, data, length, this, cancelControlTransfer};
}
/*
void UsbHost_Win32::Device::getDescriptor(usb::DescriptorType type, void *data, int &size) {
	int index = 0;
	int languageId = 0;
  	ULONG transferred;
	bool result = WinUsb_GetDescriptor(this->interface, int(type), index, languageId, (UCHAR*)data, size, &transferred);
	size = result ? transferred : 0;
}*/

void UsbHost_Win32::Device::activate() {
	// check if control transfer finished
	if (this->transferring) {
		DWORD transferred;
		auto result = GetOverlappedResult(this->file, &this->controlOverlapped, &transferred, false);
		if (!result) {
			auto error = GetLastError();
			if (error != ERROR_IO_INCOMPLETE) {
				// "real" error: return zero length buffer
				transferred = 0;
				result = true;

				// check if the USB device was disconnected
				if (error == ERROR_GEN_FAILURE)
					this->disconnect();
			}
		}

		if (result) {
			this->transferring = false;

			// start next control transfer operation
			this->controlWaitlist.visitSecond([this](ControlParameters &p) {
				this->transferring = true;
				WINUSB_SETUP_PACKET packet;
				packet.RequestType = UCHAR(p.requestType);
				packet.Request = p.request;
				packet.Value = p.value;
				packet.Index = p.index;
				packet.Length = p.length;
				memset(&this->controlOverlapped, 0, sizeof(OVERLAPPED));
				auto result = WinUsb_ControlTransfer(this->interface, packet, (PUCHAR)p.data, p.length, nullptr, &this->controlOverlapped);
			});

			// resume waiting coroutine			
			this->controlWaitlist.resumeFirst([transferred](ControlParameters &p) {
				//*p.size = transferred;
				return true;
			});
		}
	}
}

void UsbHost_Win32::Device::disconnect() {
	if (this->state == Device::State::CONNECTED) {
		this->flag = false;
		WinUsb_Free(this->interface);
		CloseHandle(this->file);
		this->interface = nullptr;
		this->file = INVALID_HANDLE_VALUE;	
		this->host.deviceInfos.erase(this->it);

		// change state
		this->state = Device::State::DISCONNECTED;

		// resume all coroutines waiting for transfers
		this->controlWaitlist.resumeAll([](ControlParameters &p) {
			return true;
		});
		this->transferring = false;
		for (auto &endpoint : this->bulkEndpoints) {
			endpoint.readWaitlist.resumeAll([](BulkEndpoint::ReadParameters &p) {
				*p.size = 0;
				return true;
			});
			endpoint.reading = false;
			endpoint.writeWaitlist.resumeAll([](BulkEndpoint::WriteParameters &p) {
				return true;
			});
			endpoint.writing = false;
		}

		// resume all coroutines waiting for disconnected state
		this->stateWaitlist.resumeAll([](Device::State state) {
			return state == Device::State::DISCONNECTED;
		});
	}
}


// BulkEndpoint

UsbHost_Win32::BulkEndpoint::BulkEndpoint(Device &device, int inAddress, int outAddress)
	: device(device), inAddress(inAddress), outAddress(outAddress)
{
	coco::yieldHandlers.add(*this);
	device.bulkEndpoints.add(*this);
}

UsbHost_Win32::BulkEndpoint::~BulkEndpoint() {
}

static void cancelRead(Stream::ReadParameters &p) {
	UsbHost_Win32::BulkEndpoint &endpoint = *reinterpret_cast<UsbHost_Win32::BulkEndpoint *>(p.context);

	CancelIoEx(endpoint.device.file, &endpoint.readOverlapped);
	
	// wait until cancellation completes
	DWORD transferred;
	GetOverlappedResult(endpoint.device.file, &endpoint.readOverlapped, &transferred, true);

	endpoint.reading = false;
	p.remove();
}

Awaitable<Stream::ReadParameters> UsbHost_Win32::BulkEndpoint::read(void *data, int &size) {
	if (!this->device.isConnected())
		return {};
	if (!this->reading) {
		// start reading
		this->reading = true;
		memset(&this->readOverlapped, 0, sizeof(OVERLAPPED));
		auto result = WinUsb_ReadPipe(this->device.interface, this->inAddress, (UCHAR*)data, size, nullptr, &this->readOverlapped);
	}
	return {this->readWaitlist, data, &size, this, cancelRead};
}

static void cancelWrite(Stream::WriteParameters &p) {
	UsbHost_Win32::BulkEndpoint &endpoint = *reinterpret_cast<UsbHost_Win32::BulkEndpoint *>(p.context);

	CancelIoEx(endpoint.device.file, &endpoint.writeOverlapped);
	
	// wait until cancellation completes
	DWORD transferred;
	GetOverlappedResult(endpoint.device.file, &endpoint.writeOverlapped, &transferred, true);

	endpoint.writing = false;
	p.remove();
}

Awaitable<Stream::WriteParameters> UsbHost_Win32::BulkEndpoint::write(void const *data, int size) {
	if (!this->device.isConnected())
		return {};
	if (!this->writing) {
		// start writing
		this->writing = true;
		memset(&this->writeOverlapped, 0, sizeof(OVERLAPPED));
		auto result = WinUsb_WritePipe(this->device.interface, this->outAddress, (UCHAR*)data, size, nullptr, &this->writeOverlapped);
	}
	return {this->writeWaitlist, data, size, this, cancelWrite};
}

void UsbHost_Win32::BulkEndpoint::activate() {
	// check if read finished
	if (this->reading) {
		DWORD transferred;
		auto result = GetOverlappedResult(this->device.file, &this->readOverlapped, &transferred, false);
		if (!result) {
			auto error = GetLastError();
			if (error != ERROR_IO_INCOMPLETE) {
				// "real" error: return zero length buffer
				transferred = 0;
				result = true;

				// check if the USB device was disconnected
				if (error == ERROR_GEN_FAILURE)
					this->device.disconnect();
			}
		}

		if (result) {
			this->reading = false;

			// start next read operation
			this->readWaitlist.visitSecond([this](ReadParameters &p) {
				this->reading = true;
				memset(&this->readOverlapped, 0, sizeof(OVERLAPPED));
				auto result = WinUsb_ReadPipe(this->device.interface, this->inAddress, (UCHAR*)p.data, *p.size, nullptr, &this->readOverlapped);
			});

			// resume waiting coroutine			
			this->readWaitlist.resumeFirst([transferred](ReadParameters &p) {
				*p.size = transferred;
				return true;
			});
		}
	}

	// check if write finished
	if (this->writing) {
		DWORD transferred;
		auto result = GetOverlappedResult(this->device.file, &this->writeOverlapped, &transferred, false);
		if (!result) {
			auto error = GetLastError();
			if (error != ERROR_IO_INCOMPLETE) {
				// "real" error: return zero length buffer
				transferred = 0;
				result = true;

				// check if the USB device was disconnected
				if (error == ERROR_GEN_FAILURE)
					this->device.disconnect();
			}
		}

		if (result) {
			this->writing = false;
		
			// start next write operation
			this->writeWaitlist.visitSecond([this](WriteParameters &p) {
				this->writing = true;
				memset(&this->writeOverlapped, 0, sizeof(OVERLAPPED));
				auto result = WinUsb_WritePipe(this->device.interface, this->outAddress, (UCHAR*)p.data, p.size, nullptr, &this->writeOverlapped);
			});

			// resume waiting coroutine			
			this->writeWaitlist.resumeFirst([](WriteParameters &p) {
				return true;
			});		
		}
	}
}

} // namespace coco

package ftdi

/*
#include <stdlib.h>
#include <ftdi.h>
#include <libusb.h>

#cgo pkg-config: libftdi1
//#cgo CFLAGS: -I/usr/local/include/libftdi1 -I/usr/include/libusb-1.0
//#cgo LDFLAGS: /usr/local/lib/libftdi1.a /usr/lib/x86_64-linux-gnu/libusb-1.0.a -ludev -pthread
*/
import "C"

import (
	"runtime"
	"unsafe"
)

type USBDev struct {
	Manufacturer string
	Description  string
	Serial       string
	d            *C.struct_libusb_device
}

func (u *USBDev) unref() {
	if u.d == nil {
		panic("USBDev.unref on uninitialized device")
	}
	C.libusb_unref_device(u.d)
	u.d = nil // Help GC.
}

func (u *USBDev) Close() {
	runtime.SetFinalizer(u, nil)
	u.unref()
}

// getStrings updates Manufacturer, Description, Serial strings descriptors
// in unicode form. It doesn't use ftdi_usb_get_strings because libftdi
// converts  unicode strings to ASCII.
func (u *USBDev) getStrings(dev *C.libusb_device, ds *C.struct_libusb_device_descriptor) error {
	var dh  *C.libusb_device_handle

	if e := C.libusb_open(dev, &dh); e != 0 {
		return USBError(e)
	}

	defer C.libusb_close(dh)

	langid, err := getLangId(dh)
	if err != nil {
		return err
	}

	if u.Manufacturer, err = getStringDescriptor(dh, ds.iManufacturer, langid); err != nil {
		return err
	}

	if u.Description, err = getStringDescriptor(dh, ds.iProduct, langid); err != nil {
		return err
	}

	if u.Serial, err = getStringDescriptor(dh, ds.iSerialNumber, langid); err != nil {
		return err
	}

	return nil
}

// FindAll search for all USB devices with specified vendor and  product id.
// It returns slice od found devices.
func FindAll(vendor, product int) ([]*USBDev, error) {
	if e := C.libusb_init(nil); e != 0 {
		return nil, USBError(e)
	}
	var dptr **C.struct_libusb_device
	if e := C.libusb_get_device_list(nil, &dptr); e < 0 {
		return nil, USBError(e)
	}
	defer C.libusb_free_device_list(dptr, 1)
	devs := (*[1 << 28]*C.libusb_device)(unsafe.Pointer(dptr))

	var n int
	for i, dev := range devs[:] {
		if dev == nil {
			n = i
			break
		}
	}
	descr := make([]*C.struct_libusb_device_descriptor, n)
	for i, dev := range devs[:n] {
		var ds C.struct_libusb_device_descriptor
		if e := C.libusb_get_device_descriptor(dev, &ds); e < 0 {
			return nil, USBError(e)
		}
		if int(ds.idVendor) == vendor && int(ds.idProduct) == product {
			descr[i] = &ds
			continue
		}
		if vendor == 0 && product == 0 && ds.idVendor == 0x403 {
			switch ds.idProduct {
			case 0x6001, 0x6010, 0x6011, 0x6014, 0x6015:
				descr[i] = &ds
				continue
			}
		}
		n--
	}
	if n == 0 {
		return nil, nil
	}
	ret := make([]*USBDev, n)
	n = 0
	for i, ds := range descr {
		if ds == nil {
			continue
		}
		u := new(USBDev)
		u.d = devs[i]
		C.libusb_ref_device(u.d)
		runtime.SetFinalizer(u, (*USBDev).unref)
		if err := u.getStrings(u.d, ds); err != nil {
			return nil, err
		}
		ret[n] = u
		n++
	}
	return ret, nil
}

// OpenUSBDev opens channel (interface) c of USB device u.
// u must be FTDI device.
func OpenUSBDev(u *USBDev, c Channel) (*Device, error) {
	d, err := makeDevice(c)
	if err != nil {
		return nil, err
	}

	if e := C.ftdi_usb_open_dev(d.ctx, u.d); e < 0 {
		defer d.free()
		return nil, d.makeError(e)
	}
	runtime.SetFinalizer(d, (*Device).close)
	return d, nil
}

func (u *USBDev) Open(c Channel) (*Device, error) {
	return OpenUSBDev(u, c)
}

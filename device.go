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
	"sync/atomic"
	"unicode/utf16"
	"unsafe"

	"github.com/pkg/errors"

	"github.com/notifai/serial"
)

func makeError(ctx *C.struct_ftdi_context, code C.int) error {
	if code >= 0 {
		return nil
	}
	return &Error{
		code: int(code),
		str:  C.GoString(C.ftdi_get_error_string(ctx)),
	}
}

func getLangId(dh *C.libusb_device_handle) (C.uint16_t, error) {
	var buf [128]C.char
	e := C.libusb_get_string_descriptor(
		dh, 0, 0,
		(*C.uchar)(unsafe.Pointer(&buf[0])), C.int(len(buf)),
	)
	if e < 0 {
		return 0, USBError(e)
	}
	if e < 4 {
		return 0, errors.New("not enough data in USB language IDs descriptor")
	}
	return C.uint16_t(uint(buf[2]) | uint(buf[3])<<8), nil
}

func getStringDescriptor(dh *C.libusb_device_handle, id C.uint8_t, langid C.uint16_t) (string, error) {
	var buf [128]C.char
	e := C.libusb_get_string_descriptor(
		dh, id, C.uint16_t(langid),
		(*C.uchar)(unsafe.Pointer(&buf[0])), C.int(len(buf)),
	)
	if e < 0 {
		return "", USBError(e)
	}
	if e < 2 {
		return "", errors.New("not enough data for USB string descriptor")
	}
	l := C.int(buf[0])
	if l > e {
		return "", errors.New("USB string descriptor is too short")
	}
	b := buf[2:l]
	uni16 := make([]uint16, len(b)/2)
	for i := range uni16 {
		uni16[i] = uint16(b[i*2]) | uint16(b[i*2+1])<<8
	}
	return string(utf16.Decode(uni16)), nil
}

// Device represents FTDI device.
type Device struct {
	ctx      *C.struct_ftdi_context
	acquired uintptr
	pinMask  uint16
	gpioDir  uint16
	gpioMask uint16
	gpioLo   uint8
}

// Type is numeric type id of FTDI device.
type Type uint32

const (
	TypeAM Type = iota
	TypeBM
	Type2232C
	TypeR
	Type2232H
	Type4232H
	Type232H
	Type230x
)

var types = []string{"AM", "BM", "2232C", "R", "2232H", "4232H", "232H", "230X"}

// String returns text name that describes type id.
func (t Type) String() string {
	if t >= Type(len(types)) {
		return "unknown"
	}
	return types[t]
}

// Type returns type of device d.
func (d *Device) Type() Type {
	return Type(d.ctx._type)
}

func (d *Device) acquire() bool {
	return atomic.CompareAndSwapUintptr(&d.acquired, 0, 1)
}

func (d *Device) release() {
	atomic.StoreUintptr(&d.acquired, 0)
}

func (d *Device) free() {
	if d.ctx == nil {
		panic("Device.free on uninitialized device")
	}
	C.ftdi_free(d.ctx)
	d.ctx = nil
}

func (d *Device) maxPacketSize() uint {
	return uint(d.ctx.max_packet_size)
}

func (d *Device) readRemainingData() int {
	return int(d.ctx.readbuffer_remaining)
}

func (d *Device) makeError(code C.int) error {
	return makeError(d.ctx, code)
}

func (d *Device) close() error {
	defer d.free()
	if e := C.ftdi_usb_close(d.ctx); e != 0 {
		return d.makeError(e)
	}
	return nil
}

// Close device
func (d *Device) Close() error {
	runtime.SetFinalizer(d, nil)
	return d.close()
}

func makeDevice(c Channel) (*Device, error) {
	ctx, err := C.ftdi_new()
	if ctx == nil {
		return nil, err
	}

	d := &Device{
		ctx:      ctx,
		acquired: 0,
	}

	if c != ChannelAny {
		if e := C.ftdi_set_interface(d.ctx, uint32(c)); e < 0 {
			defer d.free()
			return nil, d.makeError(e)
		}
	}
	return d, nil
}

// Channel represents channel (interface) of FTDI device. Some devices have more
// than one channel (eg. FT2232 has 2 channels, FT4232 has 4 channels).
type Channel uint32

const (
	ChannelAny Channel = iota
	ChannelA
	ChannelB
	ChannelC
	ChannelD
)

// OpenFirst opens the first device with a given vendor and product ids. Uses
// specified channel c.
func OpenFirst(vendor, product int, c Channel) (*Device, error) {
	d, err := makeDevice(c)
	if err != nil {
		return nil, err
	}
	if e := C.ftdi_usb_open(d.ctx, C.int(vendor), C.int(product)); e < 0 {
		defer d.free()
		return nil, d.makeError(e)
	}
	runtime.SetFinalizer(d, (*Device).close)
	return d, nil
}

// Open opens the index-th device with a given vendor id, product id,
// description and serial. Uses specified channel c.
func Open(vendor, product int, description, serial string, index uint,
	c Channel) (*Device, error) {

	d, err := makeDevice(c)
	if err != nil {
		return nil, err
	}
	descr := C.CString(description)
	defer C.free(unsafe.Pointer(descr))
	ser := C.CString(serial)
	defer C.free(unsafe.Pointer(ser))

	e := C.ftdi_usb_open_desc_index(
		d.ctx,
		C.int(vendor), C.int(product),
		descr, ser,
		C.uint(index),
	)
	if e < 0 {
		defer d.free()
		return nil, d.makeError(e)
	}
	runtime.SetFinalizer(d, (*Device).close)
	return d, nil
}

// Mode represents operation mode that FTDI device can work.
type Mode byte

const (
	ModeReset Mode = (1 << iota) >> 1
	ModeBitbang
	ModeMPSSE
	ModeSyncBB
	ModeMCU
	ModeOpto
	ModeCBUS
	ModeSyncFF
	ModeFT1284
)

func (d *Device) AsSerial() (serial.Port, error) {
	var err error

	if !d.acquire() {
		return nil, errors.Errorf("ftdi: port is busy")
	}
	defer func() {
		if err != nil {
			d.release()
		}
	}()

	if err = d.SetBitmode(0, ModeReset); err != nil {
		return nil, err
	}

	port := &serialPort{dev: d}
	return port, nil
}

// SetBitmode sets operation mode for device d to mode. iomask bitmask
// configures lines corresponding to its bits as input (bit=0) or output (bit=1).
func (d *Device) SetBitmode(iomask byte, mode Mode) error {
	e := C.ftdi_set_bitmode(d.ctx, C.uchar(iomask), C.uchar(mode))
	return d.makeError(e)
}

// Reset resets device d.
func (d *Device) Reset() error {
	return d.makeError(C.ftdi_usb_reset(d.ctx))
}

// PurgeWriteBuffer clears Rx buffer (buffer for data received from USB?).
func (d *Device) PurgeWriteBuffer() error {
	return d.makeError(C.ftdi_tciflush(d.ctx))
}

// PurgeReadBuffer clears Tx buffer (buffer for data that will be sent to USB?).
func (d *Device) PurgeReadBuffer() error {
	return d.makeError(C.ftdi_tcoflush(d.ctx))
}

// PurgeBuffers clears both (Tx and Rx) buffers.
func (d *Device) PurgeBuffers() error {
	return d.makeError(C.ftdi_tcioflush(d.ctx))
}

// ReadChunkSize returns current value of read buffer chunk size.
func (d *Device) ReadChunkSize() (int, error) {
	var cs C.uint
	e := C.ftdi_read_data_get_chunksize(d.ctx, &cs)
	return int(cs), d.makeError(e)
}

// SetReadChunkSize configure read chunk size for device (default is 4096B) and
// size of software buffer dedicated for reading data from device...
func (d *Device) SetReadChunkSize(cs int) error {
	return d.makeError(C.ftdi_read_data_set_chunksize(d.ctx, C.uint(cs)))
}

// WriteChunkSize returns current value of write chunk size.
func (d *Device) WriteChunkSize() (int, error) {
	var cs C.uint
	e := C.ftdi_write_data_get_chunksize(d.ctx, &cs)
	return int(cs), d.makeError(e)
}

// SetWriteChunkSize configure write chunk size (default is 4096). If more than
// cs bytes need to be send to device, they will be split to chunks of size cs.
func (d *Device) SetWriteChunkSize(cs int) error {
	return d.makeError(C.ftdi_write_data_set_chunksize(d.ctx, C.uint(cs)))
}

// LatencyTimer returns latency timer value [ms].
func (d *Device) LatencyTimer() (int, error) {
	var lt C.uchar
	e := C.ftdi_get_latency_timer(d.ctx, &lt)
	return int(lt), d.makeError(e)
}

// SetLatencyTimer sets latency timer to lt (value beetwen 1 and 255). If FTDI
// device has fewer data to completely fill one USB packet (<62 B) it waits for
// lt ms before sending data to USB.
func (d *Device) SetLatencyTimer(lt int) error {
	return d.makeError(C.ftdi_set_latency_timer(d.ctx, C.uchar(lt)))
}

// Read reads data from device to data. It returns number of bytes read.
func (d *Device) Read(data []byte) (int, error) {
	n := C.ftdi_read_data(
		d.ctx,
		(*C.uchar)(unsafe.Pointer(&data[0])),
		C.int(len(data)),
	)
	if n < 0 {
		return 0, d.makeError(n)
	}
	return int(n), nil
}

// Write writes data from buf to device. It returns number of bytes written.
func (d *Device) Write(data []byte) (int, error) {
	n := C.ftdi_write_data(
		d.ctx,
		(*C.uchar)(unsafe.Pointer(&data[0])),
		C.int(len(data)),
	)
	if n < 0 {
		return 0, d.makeError(n)
	}
	return int(n), nil
}

// WriteString writes bytes from string s to device. It returns number of bytes written.
func (d *Device) WriteString(s string) (int, error) {
	// BUG: This will cause problems when string implementation changes.
	type stringHeader struct {
		data unsafe.Pointer
		len  int
	}
	n := C.ftdi_write_data(
		d.ctx,
		(*C.uchar)((*stringHeader)(unsafe.Pointer(&s)).data),
		C.int(len(s)),
	)
	if n < 0 {
		return 0, d.makeError(n)
	}
	return int(n), nil
}

// ReadByte reads one byte from device.
func (d *Device) ReadByte() (byte, error) {
	var b byte
	if n := C.ftdi_read_data(d.ctx, (*C.uchar)(&b), 1); n != 1 {
		return 0, d.makeError(n)
	}
	return b, nil
}

// WriteByte writes one byte to device.
func (d *Device) WriteByte(b byte) error {
	if n := C.ftdi_write_data(d.ctx, (*C.uchar)(&b), 1); n != 1 {
		return d.makeError(n)
	}
	return nil
}

// Pins returns current state of pins (circumventing the read buffer).
func (d *Device) Pins() (b byte, err error) {
	if e := C.ftdi_read_pins(d.ctx, (*C.uchar)(&b)); e != 0 {
		err = d.makeError(e)
	}
	return
}

// SetBaudrate sets the rate of data transfer.
//
// For standard USB-UART adapter it sets UART baudrate.
//
// For bitbang mode the clock is actually 16 times the br. From the FTDI
// documentation for FT232R bitbang mode:
// "The clock for the Asynchronous Bit Bang mode is actually 16 times the
// Baud rate. A value of 9600 Baud would transfer the data at (9600x16) = 153600
// bytes per second, or 1 every 6.5 μS."
//
// FT232R supports baudrates from 183.1 baud to 3 Mbaud but for real applications
// it should be <= 1 Mbaud: Actual baudrate is set to discrete value that
// satisfies the equation br = 3000000 / (n + x) where n can be an integer
// between 2 and 16384 and x can be a sub-integer of the value 0, 0.125, 0.25,
// 0.375, 0.5, 0.625, 0.75, or 0.875. When n == 1 then x should be 0, i.e.
// baud rate divisors with values between 1 and 2 are not possible.
func (d *Device) SetBaudrate(br int) error {
	return d.makeError(C.ftdi_set_baudrate(d.ctx, C.int(br)))
}

// Parity represents the parity mode
type Parity int

const (
	// ParityNone indicates no parity bit is used
	ParityNone Parity = C.NONE
	// ParityOdd indicates an odd parity bit is used
	ParityOdd Parity = C.ODD
	// ParityEven indicates an even parity bit is used
	ParityEven Parity = C.EVEN
	// ParityMark indicates that the parity bit should be a 1
	ParityMark Parity = C.MARK
	// ParitySpace indicates that the parity bit should be a 0
	ParitySpace Parity = C.SPACE
)

// DataBits represents the number of data bits to expect
type DataBits int

const (
	// DataBits7 indicates that 7 data bits are used
	DataBits7 DataBits = C.BITS_7
	// DataBits8 indicates that 8 data bits are used
	DataBits8 DataBits = C.BITS_8
)

// StopBits represents the number of stop bits to expect
type StopBits int

const (
	// StopBits1 indicates only one stop bit is expected
	StopBits1 StopBits = C.STOP_BIT_1
	// StopBits15 indicates one and a half stop bits are expected
	StopBits15 StopBits = C.STOP_BIT_15
	// StopBits2 indicates two stop bits are expected
	StopBits2 StopBits = C.STOP_BIT_2
)

// Break represents the break mode
type Break int

const (
	// BreakOff disables the use of a break signal
	BreakOff Break = C.BREAK_OFF
	// BreakOn enables the use of a break signal
	BreakOn Break = C.BREAK_ON
)

// SetLineProperties sets the uart data bit count, stop bits count, and parity mode
func (d *Device) SetLineProperties(bits DataBits, stopbits StopBits, parity Parity) error {
	e := C.ftdi_set_line_property(
		d.ctx,
		uint32(bits),
		uint32(stopbits),
		uint32(parity),
	)
	return d.makeError(e)
}

// SetLineProperties2 sets the uart data bit count, stop bits count, parity mode,
// and break usage
func (d *Device) SetLineProperties2(bits DataBits, stopbits StopBits, parity Parity, breaks Break) error {
	e := C.ftdi_set_line_property2(
		d.ctx,
		uint32(bits),
		uint32(stopbits),
		uint32(parity),
		uint32(breaks),
	)
	return d.makeError(e)
}

// FlowCtrl represents the flow control mode.
type FlowCtrl int

const (
	// FlowCtrlDisable disables all automatic flow control.
	FlowCtrlDisable FlowCtrl = (1 << iota) >> 1
	// FlowCtrlRTSCTS enables RTS CTS flow control.
	FlowCtrlRTSCTS
	// FlowCtrlDTRDSR enables DTR DSR flow control.
	FlowCtrlDTRDSR
	// FlowCtrlXONXOFF enables XON XOF flow control.
	FlowCtrlXONXOFF
)

// SetFlowControl sets the flow control parameter
func (d *Device) SetFlowControl(flowctrl FlowCtrl) error {
	return d.makeError(C.ftdi_setflowctrl(d.ctx, C.int(flowctrl)))
}

// SetDTRRTS manually sets the DTR and RTS output lines from the
// least significant bit of dtr and rts.
func (d *Device) SetDTRRTS(dtr, rts int) error {
	return d.makeError(C.ftdi_setdtr_rts(d.ctx, C.int(dtr&1), C.int(rts&1)))
}

// SetDTR manually sets the DTR output line from the least significant
// bit of dtr.
func (d *Device) SetDTR(dtr int) error {
	return d.makeError(C.ftdi_setdtr(d.ctx, C.int(dtr&1)))
}

// SetRTS manually sets the RTS output line from the least significant
// bit of rts.
func (d *Device) SetRTS(rts int) error {
	return d.makeError(C.ftdi_setrts(d.ctx, C.int(rts&1)))
}

// ChipID reads FTDI Chip-ID (not all devices support this).
func (d *Device) ChipID() (uint32, error) {
	var id C.uint
	if e := C.ftdi_read_chipid(d.ctx, &id); e < 0 {
		return 0, d.makeError(e)
	}
	return uint32(id), nil
}

// EEPROM returns a handler to the device internal EEPROM subsystem.
func (d *Device) EEPROM() EEPROM {
	return EEPROM{d}
}

type Transfer struct {
	c C.struct_ftdi_transfer_control
}

var errSubmitTransfer = errors.New("libusb_submit_transfer")

func (d *Device) SubmitRead(data []byte) (*Transfer, error) {
	tc, err := C.ftdi_read_data_submit(
		d.ctx,
		(*C.uchar)(unsafe.Pointer(&data[0])),
		C.int(len(data)),
	)
	if tc == nil {
		if err == nil {
			err = errSubmitTransfer
		}
		return nil, err
	}
	return (*Transfer)(unsafe.Pointer(tc)), nil
}

func (d *Device) SubmitWrite(data []byte) (*Transfer, error) {
	tc, err := C.ftdi_write_data_submit(
		d.ctx,
		(*C.uchar)(unsafe.Pointer(&data[0])),
		C.int(len(data)),
	)
	if tc == nil {
		if err == nil {
			err = errSubmitTransfer
		}
		return nil, err
	}
	return (*Transfer)(unsafe.Pointer(tc)), nil
}

func (d *Device) MPSSEBitDelay() float64 {
	// measured on FTDI2232H, not documented in datasheet, hence may vary
	// from on FTDI model to another...
	// left as a variable so it could be tweaked base on the FTDI bcd type,
	// the frequency, or ... whatever else
	return 0.5E-6 // seems to vary between 5 and 6.5 us
}

func (t *Transfer) Done() (int, error) {
	n := C.ftdi_transfer_data_done(&t.c)
	if n < 0 {
		return 0, makeError(t.c.ftdi, n)
	}
	return int(n), nil
}

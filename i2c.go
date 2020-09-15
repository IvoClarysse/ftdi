package ftdi

import (
	"math"
	"sync"

	"github.com/pkg/errors"
)

type i2cTimings struct {
	tHdSta float64
	tSuSta float64
	tSuSto float64
	tSuBuf float64
}

var (
	LOW                   = uint8(0x00)
	HIGH                  = uint8(0xff)
	BIT0                  = uint8(0x01)
	IDLE                  = HIGH
	SCL_BIT               = uint8(0x01)    // AD0
	SDA_O_BIT             = uint8(0x02)    // AD1
	SDA_I_BIT             = uint8(0x04)    // AD2
	SCL_FB_BIT            = uint8(0x80)    // AD7
	PAYLOAD_MAX_LENGTH    = uint64(0xFF00) // 16 bits max (- spare for control)
	HIGHEST_I2C_ADDRESS   = uint64(0x78)
	DEFAULT_BUS_FREQUENCY = float64(100000.0)
	HIGH_BUS_FREQUENCY    = float64(400000.0)
	RETRY_COUNT           = float64(3)

	I2C_MASK    = SCL_BIT | SDA_O_BIT | SDA_I_BIT
	I2C_MASK_CS = SCL_BIT | SDA_O_BIT | SDA_I_BIT | SCL_FB_BIT
	I2C_DIR     = SCL_BIT | SDA_O_BIT

	I2C_100K = i2cTimings{4.0E-6, 4.7E-6, 4.0E-6, 4.7E-6}
	I2C_400K = i2cTimings{0.6E-6, 0.6E-6, 0.6E-6, 1.3E-6}
	I2C_1M   = i2cTimings{0.26E-6, 0.26E-6, 0.26E-6, 0.5E-6}
)

var (
	i2cImmediate = []byte{byte(SEND_IMMEDIATE)}
	i2cReadBit   = []byte{byte(READ_BITS_PVE_MSB), 0}
	i2cReadByte  = []byte{byte(READ_BYTES_PVE_MSB), 0, 0}
	i2cWriteByte = []byte{byte(WRITE_BYTES_NVE_MSB), 0, 0}
	i2cNack      = []byte{byte(WRITE_BITS_NVE_MSB), 0, HIGH}
	i2cAck       = []byte{byte(WRITE_BITS_NVE_MSB), 0, LOW}
)

type I2CError string

var (
	ErrI2CGPIO    = I2CError("ftdi: cannot read gpio")
	ErrI2CIO      = I2CError("ftdi: i2c io error")
	ErrI2CNAck    = I2CError("ftdi: i2c nack")
	ErrI2CTimeout = I2CError("ftdi: i2c timeout")
)

func (e I2CError) Error() string {
	return string(e)
}

type I2CPort struct {
	addr uint16
}

type i2cOptions struct {
	addr       *uint8
	relax      bool
	retryCount int
}

func newI2COptions(opts ...I2COption) (*i2cOptions, error) {
	res := &i2cOptions{
		relax: true,
	}

	if err := res.apply(opts...); err != nil {
		return nil, err
	}
	return res, nil
}

func (opts *i2cOptions) apply(opt ...I2COption) error {
	for _, option := range opt {
		if err := option(opts); err != nil {
			return err
		}
	}

	return nil
}

type I2COption func(*i2cOptions) error

func WithAddress(addr uint8) I2COption {
	return func(options *i2cOptions) error {
		val := new(uint8)
		*val = addr

		options.addr = val

		return nil
	}
}

func WithRelax(val bool) I2COption {
	return func(options *i2cOptions) error {
		options.relax = val

		return nil
	}
}

type I2CController struct {
	mu           sync.Mutex
	ftdi         *Device
	fakeTriState bool
	widePort     bool
	retryCount   int
	i2cMask      uint16
	gpioDir      uint16
	gpioMask     uint16
	gpioLo       uint8
	rxSize       uint8
	txSize       uint8
	ckHdSta      uint8
	ckSuSto      uint8
	ckIdle       uint8
	ckDelay      uint8
}

func (ctrl *I2CController) Read(data []byte, opts ...I2COption) (int, error) {
	ctrl.mu.Lock()
	defer ctrl.mu.Unlock()

	doEpilogue := true

	opt, err := newI2COptions(opts...)
	if err != nil {
		return 0, err
	}

	retryCount := opt.retryCount

	for {
		if opt.addr != nil {
			addr := ((*opt.addr << 1) & HIGH) | BIT0
			if err = ctrl.doPrologue(addr); err != nil && !errors.Is(err, ErrI2CNAck) {
				return 0, err
			}
		}

		var n int
		if err == nil {
			n, err = ctrl.doRead(data)
		}

		if err != nil {
			if errors.Is(err, ErrI2CNAck) {
				if retryCount == 0 {
					return n, err
				}
				retryCount--
				continue
			}

			return n, err
		}

		doEpilogue = opt.relax

		if doEpilogue {
			if err = ctrl.doEpilogue(); err != nil {
				return n, err
			}
		}
	}
}

func (ctrl *I2CController) Write(data []byte, opts ...I2COption) (int, error) {
	ctrl.mu.Lock()
	defer ctrl.mu.Unlock()

	doEpilogue := true

	opt, err := newI2COptions(opts...)
	if err != nil {
		return 0, err
	}

	retryCount := opt.retryCount

	for {
		if opt.addr != nil {
			addr := (*opt.addr << 1) & HIGH
			if err = ctrl.doPrologue(addr); err != nil && !errors.Is(err, ErrI2CNAck) {
				return 0, err
			}
		}

		var n int
		if err == nil {
			n, err = ctrl.doWrite(data)
		}

		if err != nil {
			if errors.Is(err, ErrI2CNAck) {
				if retryCount == 0 {
					return n, err
				}
				retryCount--
				continue
			}

			return n, err
		}

		doEpilogue = opt.relax

		if doEpilogue {
			if err = ctrl.doEpilogue(); err != nil {
				return n, err
			}
		}
	}
}

func (ctrl *I2CController) Xfer(out []byte, in []byte, opts ...I2COption) (int, error) {
	if len(in) < 1 {
		return 0, errors.Errorf("ftdi: nothing to read")
	}

	if len(in) > int((PAYLOAD_MAX_LENGTH/3)-1) {
		return 0, errors.Errorf("ftdi: payload is too large")
	}

	ctrl.mu.Lock()
	defer ctrl.mu.Unlock()

	doEpilogue := true

	opt, err := newI2COptions(opts...)
	if err != nil {
		return 0, err
	}

	retryCount := opt.retryCount

	for {
		if opt.addr != nil {
			addr := (*opt.addr << 1) & HIGH
			if err = ctrl.doPrologue(addr); err != nil && !errors.Is(err, ErrI2CNAck) {
				return 0, err
			}
		}

		var n int
		if err == nil {
			n, err = ctrl.doWrite(out)
		}

		if err == nil {
			if opt.addr != nil {
				addr := ((*opt.addr << 1) & HIGH) | BIT0
				if err = ctrl.doPrologue(addr); err != nil && !errors.Is(err, ErrI2CNAck) {
					return 0, err
				}
			}

			if err == nil {
				var nRd int
				nRd, err = ctrl.doRead(in)
				n += nRd
			}
		}

		if err != nil {
			if errors.Is(err, ErrI2CNAck) {
				if retryCount == 0 {
					return n, err
				}
				retryCount--
				continue
			}

			return n, err
		}

		doEpilogue = opt.relax

		if doEpilogue {
			if err = ctrl.doEpilogue(); err != nil {
				return n, err
			}
		}
	}
}

func (ctrl *I2CController) sendCheckAck(buf []byte) error {
	// ln := len(buf)
	if ctrl.fakeTriState {
		// SCL low, SDA high-Z (input)
		buf = append(buf, ctrl.clkLoDataInput()...)
		// read SDA (ack from slave)
		buf = append(buf, i2cReadBit...)
		// leave SCL low, restore SDA as output
		buf = append(buf, ctrl.clkLoDataHigh()...)
	} else {
		// SCL low, SDA high-Z
		buf = append(buf, ctrl.clkLoDataHigh()...)
		// read SDA (ack from slave)
		buf = append(buf, i2cReadBit...)
	}

	buf = append(buf, i2cImmediate...)
	_, err := ctrl.ftdi.Write(buf)
	if err != nil {
		return err
	}

	res := make([]byte, 1)

	if _, err = ctrl.ftdi.Read(res); err != nil {
		return err
	}

	return nil
}

func (ctrl *I2CController) direction() uint16 {
	return uint16(I2C_DIR) | ctrl.gpioDir
}

func (ctrl *I2CController) flush() error {
	if _, err := ctrl.ftdi.Write(i2cImmediate); err != nil {
		return err
	}

	if err := ctrl.ftdi.PurgeBuffers(); err != nil {
		return err
	}

	return nil
}

func (ctrl *I2CController) setGpioDir(pins, dir uint16) error {
	ctrl.mu.Lock()
	defer ctrl.mu.Unlock()

	width := uint16(8)
	if ctrl.widePort {
		width = 16
	}
	return ctrl.setGpioDirWidth(width, pins, dir)
}

func (ctrl *I2CController) setGpioDirWidth(width, pins, dir uint16) error {
	if (pins & ctrl.i2cMask) != 0 {
		return errors.Errorf("ftdi: cannot access I2C pins as GPIO")
	}

	gpioMask := uint16((1 << width) - 1)
	gpioMask &= ^ctrl.i2cMask

	if (pins & gpioMask) != pins {
		return errors.Errorf("ftdi: no such GPIO pin(s)")
	}

	ctrl.gpioDir &= ^pins
	ctrl.gpioDir |= pins & dir
	ctrl.gpioMask = gpioMask & pins
	return nil
}

func (ctrl *I2CController) dataLo() []byte {
	res := []byte{
		uint8(SET_BITS_LOW),
		SCL_BIT | ctrl.gpioLo,
		I2C_DIR | uint8(ctrl.gpioDir&0xFF),
	}

	return res
}

func (ctrl *I2CController) clkLoDataHigh() []byte {
	res := []byte{
		uint8(SET_BITS_LOW),
		SDA_O_BIT | ctrl.gpioLo,
		I2C_DIR | uint8(ctrl.gpioDir&0xFF),
	}
	return res
}

func (ctrl *I2CController) clkLoDataInput() []byte {
	res := []byte{
		uint8(SET_BITS_LOW),
		LOW | ctrl.gpioLo,
		SCL_BIT | uint8(ctrl.gpioDir&0xFF),
	}
	return res
}

func (ctrl *I2CController) clkLoDataLo() []byte {
	res := []byte{
		uint8(SET_BITS_LOW),
		ctrl.gpioLo,
		I2C_DIR | uint8(ctrl.gpioDir&0xFF),
	}
	return res
}

func (ctrl *I2CController) idle() []byte {
	res := []byte{
		uint8(SET_BITS_LOW),
		I2C_DIR | ctrl.gpioLo,
		I2C_DIR | uint8(ctrl.gpioDir&0xFF),
	}

	return res
}

func (ctrl *I2CController) start() []byte {
	dataLo := ctrl.dataLo()
	clkLoDataLo := ctrl.clkLoDataLo()

	var res []byte

	for i := uint8(0); i < ctrl.ckHdSta; i++ {
		res = append(res, dataLo...)
	}

	for i := uint8(0); i < ctrl.ckHdSta; i++ {
		res = append(res, clkLoDataLo...)
	}

	return res
}

func (ctrl *I2CController) stop() []byte {
	dataHi := ctrl.clkLoDataHigh()
	dataLo := ctrl.dataLo()
	idle := ctrl.idle()

	var res []byte

	for i := uint8(0); i < ctrl.ckHdSta; i++ {
		res = append(res, dataHi...)
	}

	for i := uint8(0); i < ctrl.ckSuSto; i++ {
		res = append(res, dataLo...)
	}

	for i := uint8(0); i < ctrl.ckIdle; i++ {
		res = append(res, idle...)
	}

	return res
}

func (ctrl *I2CController) computeDelayCycles(val int) float64 {
	// approx ceiling without relying on math module
	// the bit delay is far from being precisely known anyway
	bitDelay := ctrl.ftdi.MPSSEBitDelay()
	return math.Max(1.0, (float64(val)+bitDelay)/bitDelay)
}

func (ctrl *I2CController) readRaw(rdHigh bool) ([]byte, error) {
	var cmd []byte
	var res []byte

	cmd = append(cmd, uint8(GET_BITS_LOW))
	if rdHigh {
		cmd = append(cmd, uint8(GET_BITS_HIGH))
		res = make([]byte, 2)
	} else {
		res = make([]byte, 1)
	}
	cmd = append(cmd, uint8(SEND_IMMEDIATE))

	if _, err := ctrl.ftdi.Write(cmd); err != nil {
		return nil, err
	}

	n, err := ctrl.ftdi.Read(res)
	if err != nil {
		return nil, err
	}
	if n != cap(res) {
		return nil, ErrI2CGPIO
	}

	return res, nil
}

func (ctrl *I2CController) writeRaw(data uint16, wrHigh bool) (int, error) {
	dir := ctrl.direction()
	var cmd []byte
	cmd = append(cmd, uint8(SET_BITS_LOW))
	cmd = append(cmd, uint8(data&0xFF))
	cmd = append(cmd, uint8(dir&0xFF))

	if wrHigh {
		cmd = append(cmd, uint8(SET_BITS_HIGH))
		cmd = append(cmd, uint8((data>>8)&0xFF))
		cmd = append(cmd, uint8((dir>>8)&0xFF))
	}

	return ctrl.ftdi.Write(cmd)
}

func (ctrl *I2CController) doPrologue(addr uint8) error {
	buf := ctrl.idle()
	buf = append(buf, ctrl.start()...)
	buf = append(buf, i2cWriteByte...)
	buf = append(buf, addr)

	return ctrl.sendCheckAck(buf)
}

func (ctrl *I2CController) doEpilogue() error {
	buf := ctrl.stop()

	_, _ = ctrl.ftdi.Write(buf)
	_, _ = ctrl.ftdi.ReadByte()

	return nil
}

func (ctrl *I2CController) doRead(to []byte) (int, error) {
	if len(to) == 0 {
		// force a real read request on device, but discard any result
		buf := make([]byte, 1)

		if _, err := ctrl.ftdi.Write(buf); err != nil {
			return 0, err
		}

		if _, err := ctrl.ftdi.Read([]byte{}); err != nil {
			return 0, err
		}

		return 0, nil
	}

	var readLast []byte
	var readNotLast []byte

	if ctrl.fakeTriState {
		readByte := ctrl.clkLoDataInput()
		readByte = append(readByte, i2cReadByte...)
		readByte = append(readByte, ctrl.clkLoDataHigh()...)

		readNotLast = append(readNotLast, readByte...)
		readNotLast = append(readNotLast, i2cAck...)
		clkLoDataLo := ctrl.clkLoDataLo()
		for i := uint8(0); i < ctrl.ckDelay; i++ {
			readNotLast = append(readNotLast, clkLoDataLo...)
		}

		readLast = append(readLast, i2cReadByte...)
		readLast = append(readLast, i2cNack...)
		clkLoDataHi := ctrl.clkLoDataHigh()
		for i := uint8(0); i < ctrl.ckDelay; i++ {
			readLast = append(readLast, clkLoDataHi...)
		}
	} else {
		readNotLast = append(readNotLast, i2cReadByte...)
		readNotLast = append(readNotLast, i2cAck...)
		clkLoDataHi := ctrl.clkLoDataHigh()
		for i := uint8(0); i < ctrl.ckDelay; i++ {
			readNotLast = append(readNotLast, clkLoDataHi...)
		}

		readLast = append(readLast, i2cReadByte...)
		readLast = append(readLast, i2cNack...)
		for i := uint8(0); i < ctrl.ckDelay; i++ {
			readLast = append(readLast, clkLoDataHi...)
		}
	}

	// maximum RX size to fit in FTDI FIFO, minus 2 status bytes
	chunkSize := int(ctrl.rxSize - 2)

	// limit RX chunk size to the count of I2C packable commands in the FTDI
	// TX FIFO (minus one byte for the last 'send immediate' command)
	txCount := int(ctrl.txSize - 1) // cmdSize
	if txCount < chunkSize {
		chunkSize = txCount
	}

	rem := len(to)

	cmd := make([]byte, len(readNotLast)*chunkSize)
	for i := 0; i < chunkSize; i++ {
		cmd = append(cmd, readNotLast...)
	}

	readSz := chunkSize
	offset := 0

	for rem > 0 {
		if rem <= chunkSize {
			cmd = make([]byte, len(readNotLast)*(rem-1)+len(readLast)+len(i2cImmediate))
			for i := 0; i < (rem - 1); i++ {
				cmd = append(cmd, readNotLast...)
			}

			cmd = append(cmd, readLast...)
			cmd = append(cmd, i2cImmediate...)
			readSz = rem
		}

		if _, err := ctrl.ftdi.Write(cmd); err != nil {
			return len(to) - rem, err
		}

		n, err := ctrl.ftdi.Read(to[offset : offset+readSz])
		offset += n
		rem -= n

		if err != nil {
			return len(to) - rem, err
		}
	}

	return len(to), nil
}

func (ctrl *I2CController) doWrite(buf []byte) (int, error) {
	for i, b := range buf {
		res := i2cWriteByte
		res = append(res, b)
		if err := ctrl.sendCheckAck(res); err != nil {
			return i, err
		}
	}

	return len(buf), nil
}

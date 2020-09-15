package ftdi

type mpsseCommand byte
type ftdiMpsseCommand byte

// MPSSE Commands
const (
	WRITE_BYTES_PVE_MSB  = mpsseCommand(0x10)
	WRITE_BYTES_NVE_MSB  = mpsseCommand(0x11)
	WRITE_BITS_PVE_MSB   = mpsseCommand(0x12)
	WRITE_BITS_NVE_MSB   = mpsseCommand(0x13)
	WRITE_BYTES_PVE_LSB  = mpsseCommand(0x18)
	WRITE_BYTES_NVE_LSB  = mpsseCommand(0x19)
	WRITE_BITS_PVE_LSB   = mpsseCommand(0x1a)
	WRITE_BITS_NVE_LSB   = mpsseCommand(0x1b)
	READ_BYTES_PVE_MSB   = mpsseCommand(0x20)
	READ_BYTES_NVE_MSB   = mpsseCommand(0x24)
	READ_BITS_PVE_MSB    = mpsseCommand(0x22)
	READ_BITS_NVE_MSB    = mpsseCommand(0x26)
	READ_BYTES_PVE_LSB   = mpsseCommand(0x28)
	READ_BYTES_NVE_LSB   = mpsseCommand(0x2c)
	READ_BITS_PVE_LSB    = mpsseCommand(0x2a)
	READ_BITS_NVE_LSB    = mpsseCommand(0x2e)
	RW_BYTES_PVE_NVE_MSB = mpsseCommand(0x31)
	RW_BYTES_NVE_PVE_MSB = mpsseCommand(0x34)
	RW_BITS_PVE_PVE_MSB  = mpsseCommand(0x32)
	RW_BITS_PVE_NVE_MSB  = mpsseCommand(0x33)
	RW_BITS_NVE_PVE_MSB  = mpsseCommand(0x36)
	RW_BITS_NVE_NVE_MSB  = mpsseCommand(0x37)
	RW_BYTES_PVE_NVE_LSB = mpsseCommand(0x39)
	RW_BYTES_NVE_PVE_LSB = mpsseCommand(0x3c)
	RW_BITS_PVE_PVE_LSB  = mpsseCommand(0x3a)
	RW_BITS_PVE_NVE_LSB  = mpsseCommand(0x3b)
	RW_BITS_NVE_PVE_LSB  = mpsseCommand(0x3e)
	RW_BITS_NVE_NVE_LSB  = mpsseCommand(0x3f)
	WRITE_BITS_TMS_PVE   = mpsseCommand(0x4a)
	WRITE_BITS_TMS_NVE   = mpsseCommand(0x4b)
	RW_BITS_TMS_PVE_PVE  = mpsseCommand(0x6a)
	RW_BITS_TMS_PVE_NVE  = mpsseCommand(0x6b)
	RW_BITS_TMS_NVE_PVE  = mpsseCommand(0x6e)
	RW_BITS_TMS_NVE_NVE  = mpsseCommand(0x6f)
	SEND_IMMEDIATE       = mpsseCommand(0x87)
	WAIT_ON_HIGH         = mpsseCommand(0x88)
	WAIT_ON_LOW          = mpsseCommand(0x89)
	READ_SHORT           = mpsseCommand(0x90)
	READ_EXTENDED        = mpsseCommand(0x91)
	WRITE_SHORT          = mpsseCommand(0x92)
	WRITE_EXTENDED       = mpsseCommand(0x93)
	// -H series only
	DISABLE_CLK_DIV5 = mpsseCommand(0x8a)
	ENABLE_CLK_DIV5  = mpsseCommand(0x8b)
)

// FTDI MPSSE commands
const (
	SET_BITS_LOW    = ftdiMpsseCommand(0x80) // Change LSB GPIO output
	SET_BITS_HIGH   = ftdiMpsseCommand(0x82) // Change MSB GPIO output
	GET_BITS_LOW    = ftdiMpsseCommand(0x81) // Get LSB GPIO output
	GET_BITS_HIGH   = ftdiMpsseCommand(0x83) // Get MSB GPIO output
	LOOPBACK_START  = ftdiMpsseCommand(0x84) // Enable loopback
	LOOPBACK_END    = ftdiMpsseCommand(0x85) // Disable loopback
	SET_TCK_DIVISOR = ftdiMpsseCommand(0x86) // Set clock
	// -H series only
	ENABLE_CLK_3PHASE      = ftdiMpsseCommand(0x8c) // Enable 3-phase data clocking (I2C)
	DISABLE_CLK_3PHASE     = ftdiMpsseCommand(0x8d) // Disable 3-phase data clocking
	CLK_BITS_NO_DATA       = ftdiMpsseCommand(0x8e) // Allows JTAG clock to be output w/o data
	CLK_BYTES_NO_DATA      = ftdiMpsseCommand(0x8f) // Allows JTAG clock to be output w/o data
	CLK_WAIT_ON_HIGH       = ftdiMpsseCommand(0x94) // Clock until GPIOL1 is high
	CLK_WAIT_ON_LOW        = ftdiMpsseCommand(0x95) // Clock until GPIOL1 is low
	ENABLE_CLK_ADAPTIVE    = ftdiMpsseCommand(0x96) // Enable JTAG adaptive clock for ARM
	DISABLE_CLK_ADAPTIVE   = ftdiMpsseCommand(0x97) // Disable JTAG adaptive clock
	CLK_COUNT_WAIT_ON_HIGH = ftdiMpsseCommand(0x9c) // Clock byte cycles until GPIOL1 is high
	CLK_COUNT_WAIT_ON_LOW  = ftdiMpsseCommand(0x9d) // Clock byte cycles until GPIOL1 is low
	// FT232H only
	DRIVE_ZERO = ftdiMpsseCommand(0x9e) // Drive-zero mode
)

package ftdi

import (
	"time"

	"github.com/notifai/serial"
)

type serialPort struct {
	dev *Device
}

var _ serial.Port = (*serialPort)(nil)

func (s *serialPort) Read(p []byte) (n int, err error) {
	n, err = s.dev.Read(p)
	return
}

func (s *serialPort) Write(p []byte) (n int, err error) {
	n, err = s.dev.Write(p)
	return
}

func (s *serialPort) Close() error {
	return nil
}

func (s *serialPort) Flush() error {
	return s.dev.PurgeBuffers()
}

func (s *serialPort) SetReadDeadline(time.Duration) error {
	return nil
}

func (s *serialPort) Status() (uint, error) {
	return 0, nil
}

func (s *serialPort) SetDTR(val bool) error {
	vl := 0
	if val {
		vl = 1
	}

	return s.dev.SetDTR(vl)
}

func (s *serialPort) SetRTS(val bool) error {
	vl := 0
	if val {
		vl = 1
	}

	return s.dev.SetRTS(vl)
}

func (s *serialPort) SetParity(_ serial.Parity) error {
	return nil
}

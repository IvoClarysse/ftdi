module github.com/ziutek/ftdi

go 1.15

require (
	github.com/pkg/errors v0.9.1
	github.com/ziutek/lcd v0.0.0-20141212131202-924f223d0903
)

replace (
	github.com/ziutek/ftdi latest => github.com/notifai/ftdi latest
)

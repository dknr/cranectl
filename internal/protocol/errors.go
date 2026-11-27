package protocol

import "errors"

var (
	errShortFrame = errors.New("protocol: frame too short")
	errBadMagic   = errors.New("protocol: bad magic")
	errBadLength  = errors.New("protocol: bad body length")
	errBadCRC     = errors.New("protocol: bad CRC")
)

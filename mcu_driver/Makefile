BOARD?=arduino:avr:mega
PORT?=/dev/ttyACM0

.PHONY: default compile flash clean

default: compile flash clean

compile:
	arduino-cli compile --fqbn $(BOARD) --export-binaries

flash:
	arduino-cli upload --port $(PORT) --fqbn $(BOARD) --input-dit build/arduino.avr.mega --verbose

clean:
	rm -rf build/*


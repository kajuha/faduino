ARDUINO_TYPE=arduino:avr:mega
PORT=/dev/ttyUSB0

compile:
	arduino-cli compile --fqbn ${ARDUINO_TYPE} daemon

upload:
	arduino-cli upload --fqbn ${ARDUINO_TYPE} -p ${PORT} daemon
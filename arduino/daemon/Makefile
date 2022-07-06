ARDUINO_TYPE=arduino:avr:mega
PORT=/dev/recipe.safety.faduino.dev

compile:
	arduino-cli compile --fqbn ${ARDUINO_TYPE} daemon

upload:
ifndef USER_PORT
	arduino-cli upload --fqbn ${ARDUINO_TYPE} -p ${PORT} daemon
else
	arduino-cli upload --fqbn ${ARDUINO_TYPE} -p ${USER_PORT} daemon
endif
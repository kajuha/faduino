ARDUINO_TYPE=arduino:avr:mega
PORT=/dev/recipe.safety.faduino.dev

compile:
	arduino-cli compile --fqbn ${ARDUINO_TYPE} daemon.ino

upload:
ifndef USER_PORT
	arduino-cli upload --fqbn ${ARDUINO_TYPE} -p ${PORT} daemon.ino
else
	arduino-cli upload --fqbn ${ARDUINO_TYPE} -p ${USER_PORT} daemon.ino
endif
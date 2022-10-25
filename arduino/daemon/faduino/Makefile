ARDUINO_TYPE=arduino:avr:mega
PORT=/dev/recipe.safety.faduino.dev
LIBRARY=../include

compile:
	arduino-cli compile --fqbn ${ARDUINO_TYPE} --library ${LIBRARY} faduino.ino

upload:
ifndef USER_PORT
	arduino-cli upload --fqbn ${ARDUINO_TYPE} -p ${PORT} faduino.ino
else
	arduino-cli upload --fqbn ${ARDUINO_TYPE} -p ${USER_PORT} faduino.ino
endif
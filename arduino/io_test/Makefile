ARDUINO_TYPE=arduino:avr:mega
PORT=/dev/recipe.safety.faduino.dev

compile:
	arduino-cli compile --fqbn ${ARDUINO_TYPE} io_test.ino

upload:
ifndef USER_PORT
	arduino-cli upload --fqbn ${ARDUINO_TYPE} -p ${PORT} io_test.ino
else
	arduino-cli upload --fqbn ${ARDUINO_TYPE} -p ${USER_PORT} io_test.ino
endif
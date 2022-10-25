ARDUINO_TYPE=arduino:avr:mega
PORT=/dev/recipe.safety.faduino.dev
LIBRARY=../include
INO=ioboard_v1_2.ino

compile:
	arduino-cli compile --fqbn ${ARDUINO_TYPE} --library ${LIBRARY} ${INO}

upload:
ifndef USER_PORT
	arduino-cli upload --fqbn ${ARDUINO_TYPE} -p ${PORT} ${INO}
else
	arduino-cli upload --fqbn ${ARDUINO_TYPE} -p ${USER_PORT} ${INO}
endif
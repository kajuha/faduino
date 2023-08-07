SERIAL_PORT=/dev/recipe.safety.faduino
TCP_PORT=20212

USER_NAME=recipelab

all: main.cpp serial.cpp reprintf.cpp
	g++ -o faduinod main.cpp serial.cpp reprintf.cpp -lboost_thread -lboost_system -lpthread -Wall `pkg-config --cflags --libs serial`

run:
ifndef USER_SERIAL_PORT
	@echo ${SERIAL_PORT}
	/home/${USER_NAME}/catkin_ws/src/faduino/daemon/faduinod ${SERIAL_PORT} ${TCP_PORT}
else
	@echo ${USER_SERIAL_PORT}
	/home/${USER_NAME}/catkin_ws/src/faduino/daemon/faduinod ${USER_SERIAL_PORT} ${TCP_PORT}
endif

install: all
	sudo ln -sf /home/${USER_NAME}/catkin_ws/src/faduino/daemon/systemd/faduinod.service /etc/systemd/system
	# sudo systemctl enable faduinod
	# sudo systemctl start faduinod
	# sudo systemctl stop faduinod

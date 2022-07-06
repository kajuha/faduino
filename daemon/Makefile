SERIAL_PORT=/dev/recipe.safety.faduino
TCP_PORT=20212

all:
	g++ -o faduinod main.cpp serial.cpp -lboost_thread -lboost_system -lpthread -Wall

run:
ifndef USER_SERIAL_PORT
	@echo ${SERIAL_PORT}
	/home/recipelab/catkin_ws/src/faduino/daemon/faduinod ${SERIAL_PORT} ${TCP_PORT}
else
	@echo ${USER_SERIAL_PORT}
	/home/recipelab/catkin_ws/src/faduino/daemon/faduinod ${USER_SERIAL_PORT} ${TCP_PORT}
endif

install: all
	sudo ln -sf /home/recipelab/catkin_ws/src/faduino/daemon/systemd/faduinod.service /etc/systemd/system
	# sudo systemctl enable faduinod
	# sudo systemctl start faduinod
	# sudo systemctl stop faduinod

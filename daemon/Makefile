PORT=/dev/recipe.safety.faduino

all:
	g++ -o faduinod main.cpp serial.cpp -lboost_thread -lboost_system -Wall

run:
ifndef USER_PORT
	@echo ${PORT}
	/home/kajuha/catkin_ws/src/faduino/daemon/faduinod ${PORT}
else
	@echo ${USER_PORT}
	/home/kajuha/catkin_ws/src/faduino/daemon/faduinod ${USER_PORT}
endif

install:
	sudo ln -sf /home/kajuha/catkin_ws/src/faduino/daemon/systemd/faduinod.service /etc/systemd/system
	# sudo systemctl enable faduinod
	sudo systemctl start faduinod
	# sudo systemctl stop faduinod

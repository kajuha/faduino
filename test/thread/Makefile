all:
	g++ exec.cpp -o exec -lncurses -lboost_thread -lboost_system
	g++ test_a.cpp -o test_a
	g++ test_b.cpp -o test_b

run:
	roslaunch recipelab sim_pallete.launch

# 정상동작하지 않음

# kill:
# 	./exec "kill \`ps -e|grep 'rosmaster'|awk '{print $$2}'\`"
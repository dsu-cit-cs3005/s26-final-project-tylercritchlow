# Compiler
CXX = g++
CXXFLAGS = -std=c++20 -Wall -Wextra -pedantic

# Targets
all: test_robot RobotWarz

RobotBase.o: RobotBase.cpp RobotBase.h
	$(CXX) $(CXXFLAGS) -c RobotBase.cpp

Arena.o: Arena.cpp Arena.h RobotBase.h RadarObj.h
	$(CXX) $(CXXFLAGS) -c Arena.cpp

RobotWarz.o: RobotWarz.cpp Arena.h
	$(CXX) $(CXXFLAGS) -c RobotWarz.cpp

test_robot: test_robot.cpp RobotBase.o
	$(CXX) $(CXXFLAGS) test_robot.cpp RobotBase.o -ldl -o test_robot

RobotWarz: RobotWarz.o Arena.o RobotBase.o
	$(CXX) $(CXXFLAGS) RobotWarz.o Arena.o RobotBase.o -ldl -o RobotWarz

clean:
	rm -f *.o test_robot RobotWarz robots/libRobot_*.so

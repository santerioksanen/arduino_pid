g++ -std=c++11 -DTEST -Wall -o test_pid ../pid_controller.cpp test_pid.cpp && ./test_pid --success
g++ -std=c++11 -DTEST -Wall -o test_odometry ../odometry.cpp test_odometry.cpp && ./test_odometry --success

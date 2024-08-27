# copy compiled files to bin

mkdir -p /home/thunder_dev/thunder_dynamics/bin && cd /home/thunder_dev/thunder_dynamics/bin && \
	cp -f /home/thunder_dev/thunder_dynamics/src/thunder/build/thunder ./thunder && \
	mkdir -p neededFiles && cd neededFiles && \
	cp -f /home/thunder_dev/thunder_dynamics/src/thunder/thunder_robot_template/thunder_robot.h ./thunder_robot_template.h && \
	cp -f /home/thunder_dev/thunder_dynamics/src/thunder/thunder_robot_template/thunder_robot.cpp ./thunder_robot_template.cpp

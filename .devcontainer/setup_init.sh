# init container script, launched in .devcontainer.json

mkdir -p /home/thunder_dev/thunder_dynamics/bin && cd /home/thunder_dev/thunder_dynamics/bin && \
	cp -f /home/thunder_dynamics_tmp/src/thunder/build/thunder ./thunder && \
	mkdir -p neededFiles && cd neededFiles && \
	cp -f /home/thunder_dynamics_tmp/src/thunder_robot/library/thunder_robot.h ./thunder_robot.h && \
	cp -f /home/thunder_dynamics_tmp/src/thunder_robot/src/thunder_robot.cpp ./thunder_robot.cpp
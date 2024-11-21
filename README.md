# Pi Sonar Node

Sonar Sensor Handling based on `pigpio` library.

To build:

```
cd ~/catkin_ws/src
git clone https://github.com/UbiquityRobotics/pi_sonar.git
cd ..
colcon build
source install/setup.bash
```

The node needs to access the pigpio daemon, hence the following steps to have it run at startup (if it does not do so already):

```
git clone https://raw.githubusercontent.com/joan2937/pigpio.git
cd pigpio
make
sudo make install

sudo pigpiod
```

To run:

```
ros2 launch pi_sonar ubiquity_sonar.launch.py
```

## For developers only
Update pigpio:

git subtree pull --squash --prefix=pigpio https://github.com/joan2937/pigpio.git {ref to pull}

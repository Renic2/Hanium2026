# EBIMU service

The deployed service enables only the installed UART2 sensor:

- `/dev/ttyS2` -> `/dev/imu_left` -> `imu_left_link`

UART3/right is disabled and is neither mapped into the container nor required
by the healthcheck. At every serial connection the node asks the EBIMU-9DOFV6
for ASCII quaternion,
angular velocity, and raw acceleration (including gravity) at 100 Hz. The
quaternion order is converted from the EBIMU's `z,y,x,w` order to ROS
`x,y,z,w`; degrees/s and g are converted to rad/s and m/s^2.

The two frame IDs identify the native sensor coordinate frames. Do not publish
a static transform to the robot base until the remaining mounted Y/Z axis
directions and handedness have been measured; only X+ = robot-right is known.

Enabled topics:

- `/imu/left/data_raw`, `/imu/left/data`

`data_raw` marks orientation unavailable. `data` contains the normalized EBIMU
quaternion as well as the same angular velocity and acceleration values.

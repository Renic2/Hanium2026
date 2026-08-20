# EBIMU service

The deployed service enables only the installed UART2 sensor:

- `/dev/ttyS2` -> `/dev/imu_left` -> `imu_left_link`

UART3/right is disabled and is neither mapped into the container nor required
by the healthcheck. At every serial connection the node asks the EBIMU-9DOFV6
for ASCII quaternion,
angular velocity, and raw acceleration (including gravity) at 100 Hz. The
quaternion order is converted from the EBIMU's `z,y,x,w` order to ROS
`x,y,z,w`; degrees/s and g are converted to rad/s and m/s^2.

The original topics retain the native sensor coordinate frame. A startup
stationary calibration additionally rotates the vertically mounted IMU into
`base_link`, estimates gyro zero bias, normalizes the gravity magnitude, and
populates measured angular-velocity/acceleration covariance.

Enabled native topics:

- `/imu/left/data_raw`, `/imu/left/data`

Enabled calibrated topics:

- `/imu/left/data_raw_calibrated`, `/imu/left/data_calibrated`

`data_raw` marks orientation unavailable. `data` contains the normalized EBIMU
quaternion as well as the same angular velocity and acceleration values.

## Startup calibration

Every time the `imu` container starts, place the robot on level ground and keep
it completely still for at least 3 seconds. The node requires 300 consecutive
stationary samples. Motion clears the current window and calibration retries;
the native topics continue, but calibrated topics do not start until a valid
window completes.

At rest, `/imu/left/data_calibrated` should report `base_link`, angular velocity
near zero, acceleration x/y near zero, and acceleration z near `+9.80665 m/s²`.
Gravity is intentionally retained because that is the ROS IMU convention.

Gravity determines roll and pitch mounting correction, but cannot determine
yaw. `left.calibration.yaw_deg` is therefore explicitly set in `compose.yaml`
and defaults to zero. Change it only after measuring the sensor's horizontal
heading relative to robot forward.

Calibration completion and the calculated acceleration mean, gyro bias,
quaternion, and scale are available in the container log:

```bash
docker compose logs imu | grep 'IMU calibration complete'
```

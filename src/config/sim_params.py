from dataclasses import dataclass


@dataclass
class SimParams:
    dt: float = 0.005  # Timestep (200 Hz)
    duration: float = 30.0  # Default sim duration in seconds

    # IMU noise parameters (MPU-6050 approximate)
    imu_accel_noise_std: float = 0.05  # m/s^2
    imu_gyro_noise_std: float = 0.01  # rad/s
    imu_gyro_bias_std: float = 0.001  # rad/s per sqrt(s) (random walk)

    # Ultrasonic noise parameters (HC-SR04)
    ultrasonic_noise_std: float = 0.02  # meters

    # Sensor rates
    imu_rate_hz: float = 200.0  # IMU runs every step
    ultrasonic_rate_hz: float = 20.0  # Ultrasonic at 20 Hz
    lidar_rate_hz: float = 10.0  # LiDAR at 10 Hz

    # EKF tuning
    ekf_process_noise_pos: float = 0.01  # Position process noise
    ekf_process_noise_vel: float = 0.1  # Velocity process noise
    ekf_process_noise_quat: float = 0.001  # Quaternion process noise
    ekf_process_noise_bias: float = 0.0001  # Gyro bias process noise
    ekf_accel_measurement_noise: float = 0.5  # Accelerometer measurement noise
    ekf_altitude_measurement_noise: float = 0.05  # Ultrasonic measurement noise
    ekf_innovation_threshold: float = 5.0  # Innovation gate (std devs)

    # Control defaults
    attitude_kp: float = 8.0
    attitude_ki: float = 0.5
    attitude_kd: float = 3.0
    altitude_kp: float = 5.0
    altitude_ki: float = 1.0
    altitude_kd: float = 3.0
    position_kp: float = 1.0
    position_ki: float = 0.1
    position_kd: float = 0.8
    max_tilt_angle: float = 0.524  # ~30 degrees in radians

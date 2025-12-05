from sense_hat import SenseHat
import time
import numpy as np
import math

sense = SenseHat()
sense.set_imu_config(False, True, True)
position = np.array([0.0, 0.0, 0.0])
velocity = np.array([0.0, 0.0, 0.0])
interval = 1/20.0  # 20 Hz
last_time = time.time()
while True:
    while time.time() - last_time < interval:
        pass
    acclelerations = np.array(list(sense.get_accelerometer_raw().values()))
    rotations = np.array(list(sense.get_gyroscope().values()))

    last_time = time.time()

    acclelerations = np.round(acclelerations, decimals=2)
    rotations = np.round(rotations, decimals=2)


    # Rotate acceleration vector based on gyroscope data
    cos_pitch = math.cos(math.radians(rotations[0]))
    sin_pitch = math.sin(math.radians(rotations[0]))
    cos_roll = math.cos(math.radians(rotations[1]))
    sin_roll = math.sin(math.radians(rotations[1]))
    cos_yaw = math.cos(math.radians(rotations[2]))
    sin_yaw = math.sin(math.radians(rotations[2]))

    R_x = np.array([[1, 0, 0],
                    [0, cos_roll, -sin_roll],
                    [0, sin_roll, cos_roll]])
    R_y = np.array([[cos_pitch, 0, sin_pitch],
                    [0, 1, 0],
                    [-sin_pitch, 0, cos_pitch]])
    R_z = np.array([[cos_yaw, -sin_yaw, 0],
                    [sin_yaw, cos_yaw, 0],
                    [0, 0, 1]])
    R = R_x @ R_y @ R_z
    acclelerations = np.dot(R, acclelerations)

    # Cancel out gravity
    #//acclelerations[2] = acclelerations[2] - 1.0

    # Integrate acceleration to get velocity and 

    velocity = velocity + acclelerations * interval
    position = position + velocity * interval
    print(f"Position: x={position[0]:.3f}, y={position[1]:.3f}, z={position[2]:.3f}")
    print(f"Velocity: x={velocity[0]:.3f}, y={velocity[1]:.3f}, z={velocity[2]:.3f}")
    print(f"Acceleration: x={acclelerations[0]:.3f}, y={acclelerations[1]:.3f}, z={acclelerations[2]:.3f}")
    print(f"Rotation: pitch={rotations[0]:.3f}, roll={rotations[1]:.3f}, yaw={rotations[2]:.3f}")
    print("--------------------------------------------------")
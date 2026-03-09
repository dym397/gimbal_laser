from hwt905_driver import HWT905
import time

imu = HWT905("COM3",9600)

imu.open()

while True:

    acc, gyro, angle = imu.get_all()

    ax, ay, az = acc
    gx, gy, gz = gyro
    roll, pitch, yaw = angle

    print(
        f"ACC: {ax:6.2f} {ay:6.2f} {az:6.2f} | "
        f"GYRO: {gx:6.2f} {gy:6.2f} {gz:6.2f} | "
        f"ANGLE: {roll:6.2f} {pitch:6.2f} {yaw:6.2f}"
    )

    time.sleep(0.1)
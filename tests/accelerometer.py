"""
This script reads accelerometer data from the MPU6050 sensor,
averages multiple samples, and logs the averaged x, y, z values.
"""

import time
import logging
from mpu6050 import mpu6050

# Initialize logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)

# Initialize the MPU6050 sensor at I2C address 0x68
sensor = mpu6050(0x68)


def mpu6050test():
    """
    Read accelerometer data from the MPU6050 sensor,
    average over a set number of samples, and log the averaged x, y, z values.
    """
    x, y, z = 0, 0, 0
    sample_count = 10

    # Collect accelerometer data over sample_count iterations
    for _ in range(sample_count):
        accelerometer_data = sensor.get_accel_data()
        x += accelerometer_data['x']
        y += accelerometer_data['y']
        z += accelerometer_data['z']
        time.sleep(0.01)  # Small delay between samples

    # Calculate average values
    x_avg = x / sample_count
    y_avg = y / sample_count
    z_avg = z / sample_count

    # Log the averaged accelerometer values
    logging.info('X={:.3f}, Y={:.3f}, Z={:.3f}'.format(x_avg, y_avg, z_avg))


if __name__ == "__main__":
    try:
        while True:
            mpu6050test()
            time.sleep(0.3)  # Delay before the next set of samples
    except KeyboardInterrupt:
        # Exit the script gracefully on Ctrl+C
        logging.info("Script terminated by user")
    except Exception as e:
        # Log any other exceptions
        logging.error(f"An error occurred: {e}")

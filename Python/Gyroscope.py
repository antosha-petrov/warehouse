import mpu6050

class GyroscopeReader:
    def __init__(self, address):
        self.address = address
        self.sensor = mpu6050.mpu6050(self.address)


    def read_sensor_data(self):
        accelerometer_data = self.sensor.get_accel_data()
        gyroscope_data = self.sensor.get_gyro_data()
        temperature = self.sensor.get_temp()

        return accelerometer_data, gyroscope_data, temperature

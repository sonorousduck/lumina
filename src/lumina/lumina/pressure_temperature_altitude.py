import rclpy
from rclpy.node import Node

from sensor_msgs.msg import FluidPressure, Temperature
from std_msgs.msg import Float64

from .bmp388 import BMP388


# i2c interface
# Pressure range 300 … 1250 hPa
# Resolution of data: Pressure: 0.18 Pa (eqiuv. to <10 cm)
# Absolute accuracy  ±50 Pa
# Relative accuracy ± 8Pa (equiv. to ±0.6 m)
# Temperature range -40  …+85 °C

class PressureTempAltNode(Node):
    def __init__(self):
        super().__init__('pressure_alt_temp')

        self.air_publisher = self.create_publisher(FluidPressure, "/imu/air_pressure", 10)
        self.temperature_publisher = self.create_publisher(Temperature, "/imu/temperature", 10)
        self.altitude_publisher = self.create_publisher(Float64, "/imu/altitude", 10)

        self.bmp388 = BMP388()
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        temperature, pressure, altitude = self.bmp388.get_temperature_and_pressure_and_altitude()

        temperature_message = Temperature()
        fluid_message = FluidPressure()
        altitude_message = Float64()

        temperature_message.temperature = temperature / 100.0
        temperature_message.variance = 0.25 # Assuming std. dev is 0.5 degrees, so variance is 0.5 * 0.5
        fluid_message.fluid_pressure = pressure / 100.0
        fluid_message.variance = 2500.0 # Because absolute std. dev is 50. So 50 * 50 = 2500
        altitude_message.data = altitude / 100.0

        self.temperature_publisher.publish(temperature_message)
        self.air_publisher.publish(fluid_message)
        self.altitude_publisher.publish(altitude_message)



def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = PressureTempAltNode()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
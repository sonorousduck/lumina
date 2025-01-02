import rclpy
from rclpy.node import Node
from serial_motor_demo_msgs.msg import MotorCommand
from serial_motor_demo_msgs.msg import MotorVels
from serial_motor_demo_msgs.msg import EncoderVals
from std_msgs.msg import Float64
import time
import math
import serial
from threading import Lock

class ArduinoComms(Node):
    def __init__(self): 
        super().__init__("arduino_comms")
        
        # TODO: When I get my encoders, figure this value out
        self.encoder_cpr = 100
        
        self.loop_rate = 30
        self.serial_port = "/dev/ttyUSB0"
        self.baud_rate = 57600
        
        self.declare_parameter('serial_debug', value=False)
        self.debug_serial_cmds = self.get_parameter('serial_debug').value
        if (self.debug_serial_cmds):
            print("Serial debug enabled")


        self.subscription = self.create_subscription(
            MotorCommand,
            "motor_command",
            self.motor_command_callback,
            10
        )
        
        self.speed_pub = self.create_publisher(MotorVels, "motor_vels", 10)
        self.encoder_pub = self.create_publisher(EncoderVals, "encoder_vals", 10)
        self.voltage_pub = self.create_publisher(Float64, "battery/voltage", 10)
        self.battery_percent_pub = self.create_publisher(Float64, "battery/percentage", 10)
        
        self.last_enc_read_time = time.time()
        self.last_m1_enc = 0
        self.last_m2_enc = 0
        self.m1_speed = 0.0
        self.m2_speed = 0.0
        self.max_battery_charge_voltage = 8.4
        
        self.mutex = Lock()
        
        # Open Serial comms
        print(f"Connecting to port {self.serial_port} at {self.baud_rate}.")
        self.conn = serial.Serial(self.serial_port, self.baud_rate, timeout=1.0)
        print(f"Connected to {self.conn}")
        
        
    # Raw serial commands
    def send_pwm_motor_command(self, mot_1_pwm, mot_2_pwm):
        self.send_command(f"o {int(mot_1_pwm)} {int(mot_2_pwm)}")

    def send_feedback_motor_command(self, mot_1_ct_per_loop, mot_2_ct_per_loop):
        self.send_command(f"m {int(mot_1_ct_per_loop)} {int(mot_2_ct_per_loop)}")

    def send_encoder_read_command(self):
        resp = self.send_command(f"e")
        if resp:
            return [int(raw_enc) for raw_enc in resp.split()]
        return []
    
    def read_voltage(self):
        resp = self.send_command(f"v")
        if resp:
            voltage = float(resp)
            voltage_msg = Float64()
            percentage_msg = Float64()
            voltage_msg.data = voltage
            percentage_msg.data = voltage / self.max_battery_charge_voltage
            
            self.battery_percent_pub.publish(percentage_msg)
            self.voltage_pub.publish(voltage_msg)
    
    
    # More user-friendly functions

    def motor_command_callback(self, motor_command):
        if (motor_command.is_pwm):
            self.send_pwm_motor_command(motor_command.mot_1_req_rad_sec, motor_command.mot_2_req_rad_sec)
        else:
            # counts per loop = req rads/sec X revs/rad X counts/rev X secs/loop 
            scaler = (1 / (2*math.pi)) * self.encoder_cpre * (1 / self.loop_rate)
            mot1_ct_per_loop = motor_command.mot_1_req_rad_sec * scaler
            mot2_ct_per_loop = motor_command.mot_2_req_rad_sec * scaler
            self.send_feedback_motor_command(mot1_ct_per_loop, mot2_ct_per_loop)

    def check_encoders(self):
        resp = self.send_encoder_read_command()
        if (resp):

            new_time = time.time()
            time_diff = new_time - self.last_enc_read_time
            self.last_enc_read_time = new_time

            m1_diff = resp[0] - self.last_m1_enc
            self.last_m1_enc = resp[0]
            m2_diff = resp[1] - self.last_m2_enc
            self.last_m2_enc = resp[1]

            rads_per_ct = 2 * math.pi / self.encoder_cpr
            self.m1_spd = m1_diff*rads_per_ct/time_diff
            self.m2_spd = m2_diff*rads_per_ct/time_diff

            spd_msg = MotorVels()
            spd_msg.mot_1_rad_sec = self.m1_spd
            spd_msg.mot_2_rad_sec = self.m2_spd
            self.speed_pub.publish(spd_msg)

            enc_msg = EncoderVals()
            enc_msg.mot_1_enc_val = self.last_m1_enc
            enc_msg.mot_2_enc_val = self.last_m2_enc
            self.encoder_pub.publish(enc_msg)



    # Utility functions

    def send_command(self, cmd_string):
        
        self.mutex.acquire()
        try:
            cmd_string += "\r"
            self.conn.write(cmd_string.encode("utf-8"))
            if (self.debug_serial_cmds):
                print("Sent: " + cmd_string)

            ## Adapted from original
            c = ''
            value = ''
            while c != '\r':
                c = self.conn.read(1).decode("utf-8")
                if (c == ''):
                    print("Error: Serial timeout on command: " + cmd_string)
                    return ''
                value += c

            value = value.strip('\r')

            if (self.debug_serial_cmds):
                print("Received: " + value)
            return value
        finally:
            self.mutex.release()

    def close_conn(self):
        self.conn.close()


def main(args=None):
    rclpy.init(args=args)
    
    arduino_driver = ArduinoComms()
    rate = arduino_driver.create_rate(2)
    while rclpy.ok():
        rclpy.spin_once(arduino_driver)
        arduino_driver.check_encoders()
        arduino_driver.read_voltage()
        
    
    arduino_driver.close_conn()
    arduino_driver.destroy_node()
    
    rclpy.shutdown()
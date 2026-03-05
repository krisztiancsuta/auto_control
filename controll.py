from adafruit_servokit import ServoKit
import board
import busio
import adafruit_pca9685
import socket
import time
import struct
import argparse
import logging
import colorlog

class Control:
    def __init__(self,host_ip, port):
        self.host_ip=host_ip
        self.port=port
        self.define_color_logger()
        self.connect_to_server()
        self.setup_adafruit_device()

    def define_color_logger(self):
        # Create a logger
        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(logging.DEBUG)

        # Create a colorized formatter
        formatter = colorlog.ColoredFormatter(
            "%(log_color)s%(levelname)-8s%(reset)s %(blue)s%(message)s",
            datefmt=None,
            reset=True,
            log_colors={
                'DEBUG': 'cyan',
                'INFO': 'green',
                'WARNING': 'yellow',
                'ERROR': 'red',
                'CRITICAL': 'red,bg_white',
            },
            secondary_log_colors={},
            style='%'
        )

        # Create a console handler and set the formatter
        ch = logging.StreamHandler()
        ch.setFormatter(formatter)

        # Add the console handler to the logger
        self.logger.addHandler(ch)

    def connect_to_server(self):
        """
        Attempts to connect to the server continuously until successful or manually interrupted.
        Logs connection attempts and warnings.
        """
        while True:
            try:
                # Create a socket object and attempt connection
                self.logger.info(f"Try to connect {self.host_ip}  and  {self.port}")
                self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.client_socket.connect((self.host_ip, self.port))
                break  # Exit loop if connection is successful
            except ConnectionRefusedError:
                # Log warning if connection is refused and retry after a delay
                self.logger.warning(f"Could not connect to server, on {self.host_ip}  and  {self.port}, try again.")
                time.sleep(5)  # Wait for 5 seconds before retrying

    def setup_adafruit_device(self):
        """
        Initializes the Adafruit device including the I2C bus and the PCA9685 PWM driver.
        Sets the PWM frequency to 250 Hz.
        Initializes servo motors and sets their parameters.
        Sends 'ready' message to the client socket.
        """
        self.logger.info(f"Founded board: {board.SCL}, {board.SDA}-")  # Log the board information
        i2c = busio.I2C(board.SCL, board.SDA)  # Initialize the I2C bus
        pca = adafruit_pca9685.PCA9685(i2c)  # Initialize the PCA9685 PWM driver

        # Set the PWM frequency to 250 Hz (standard for ESCs)
        pca.frequency = 250

        self.myKit = ServoKit(channels=16)  # Initialize the ServoKit with 16 channels
        angle = 0  # Initialize angle variable
        speed = 0.0  # Initialize speed variable
        self.myKit.servo[4].actuation_range = 180  # Set actuation range for servo

        self.myKit.servo[4].set_pulse_width_range(400, 2000)  # Set pulse width range for servo
        self.client_socket.send('ready'.encode())  # Send 'ready' message to the client
        self.logger.info("Ready to go!")  # Log ready status

    def run(self):
        """
        Runs the control loop to receive data from the client socket, decode it, and control servo motors accordingly.
        Handles exceptions and sets servo motors to default position and throttle in case of connection loss.
        """
        try:
            while True:
                try:
                    data = self.client_socket.recv(1024)  # Receive data from client
                except socket.error:
                    self.logger.critical("Connection lost")  # Log connection loss
                    self.myKit.servo[4].angle = 50  # Set servo angle to default position
                    self.myKit.continuous_servo[8].throttle = 0.1  # Set continuous servo throttle to default
                    break  # Exit loop
                if not data:
                    self.logger.critical("Connection lost")  # Log connection loss
                    self.myKit.servo[4].angle = 50  # Set servo angle to default position
                    self.myKit.continuous_servo[8].throttle = 0.1  # Set continuous servo throttle to default
                    break  # Exit loop
                decoded = data.decode('utf8').split()  # Decode received data
                if (len(decoded[1].split('-')) > 1):  # Check for data value error
                    self.logger.error("Data value error")  # Log data value error
                else:
                    speed = float(decoded[0])  # Get speed value
                    angle = float(decoded[1])  # Get angle value
                if (angle < -0.5 or angle > 100.5):  # Check if angle is out of bounds
                    angle = 50  # Set angle to default position
                if (speed > 0.5 or speed < -0.5):  # Check if speed is out of bounds
                    speed = 0.1  # Set speed to default value

                self.myKit.servo[4].angle = angle  # Set servo angle
                self.myKit.continuous_servo[8].throttle = speed  # Set continuous servo throttle
        finally:
            self.myKit.servo[4].angle = 50  # Set servo angle to default position
            self.myKit.continuous_servo[8].throttle = 0.1  # Set continuous servo throttle to default


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Initialize with config file")
    parser.add_argument("--host_ip", type=str, help="Remote computer IP", required=True)
    parser.add_argument("--host_port", type=int, required=True)
    args = parser.parse_args()
    auto= Control(host_ip=args.host_ip, port=args.host_port)
    auto.run()









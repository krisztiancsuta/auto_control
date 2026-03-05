import socket
import logging
import colorlog
import keyboard
import time
import argparse

class HandleKey:
    """
    Class to handle keyboard inputs and send corresponding commands to a remote car.
    """

    def __init__(self, host_ip, port):
        """
        Initialize the HandleKey instance.

        Args:
            host_ip (str): IP address of the remote computer.
            port (int): Port number to establish connection.
        """
        self.define_color_logger()
        self.host_ip = host_ip
        self.port = port
        self.connect_to_the_car()

        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()

    def connect_to_the_car(self):
        """
        Connect to the car via socket connection.
        """
        self.serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.serversocket.bind((self.host_ip, self.port))
        self.serversocket.listen(1)
        self.logger.info(f'Server listening on {self.host_ip},{self.port}')

        self.client_socket, self.cilent_addr = self.serversocket.accept()
        self.logger.info(f'Got a connection from {self.cilent_addr}')

    def define_color_logger(self):
        """
        Define a colorized logger for debugging.
        """
        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(logging.DEBUG)

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

        ch = logging.StreamHandler()
        ch.setFormatter(formatter)

        self.logger.addHandler(ch)

    def on_press(self, key):
        """
        Callback function when a key is pressed.
        """
        try:
            if key.char == 'w':
                self.press_w = True
            if key.char == 'a':
                self.press_a = True
            if key.char == 's':
                self.press_s = True
            if key.char == 'd':
                self.press_d = True
            if key.char == 'p':
                self.press_p = True
        except AttributeError as e:
            self.logger.warning(f'special key pressed: {e}')

    def on_release(self, key):
        """
        Callback function when a key is released.
        """
        try:
            if key.char == 'w':
                self.press_w = False
            if key.char == 'a':
                self.press_a = False
            if key.char == 's':
                self.press_s = False
            if key.char == 'd':
                self.press_d = False
            if key.char == 'p':
                self.press_p = False
        except AttributeError as e:
            self.logger.warning(f'special key pressed: {e}')

    def calculate(self):
        """
        Calculate the speed and angle based on pressed keys.
        """
        if self.press_w:
            self.speed = self.max_speed
        if self.press_s:
            self.speed = self.min_speed
        if self.press_a:
            self.angle = 0
        if self.press_d:
            self.angle = 100
        if self.press_p:
            self.enable = 1
        if not (self.press_a or self.press_d):
            self.angle = 50
        if not (self.press_w or self.press_s):
            if self.speed > 0.1:
                self.speed -= 0.02
            if self.speed < 0.1:
                self.speed += 0.02
        if not self.press_p:
            self.enable = 0

    def send(self):
        """
        Send speed and angle information to the connected client.
        """
        message = f"{self.speed}  {self.angle}"
        self.client_socket.send(message.encode('utf-8'))

    def run(self):
        """
        Start the communication with the client and handle key events.
        """
        try:
            ready_signal = self.client_socket.recv(1024).decode()
            if ready_signal == 'ready':
                self.logger.info('Client is ready')
            while True:
                time.sleep(0.1)
                self.calculate()
                self.send()

        finally:
            self.logger.error("Something went wrong, close the communication.")
            self.serversocket.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Initialize with config file")
    parser.add_argument("--host_ip", type=str, help="Remote computer IP", default="0.0.0.0")
    parser.add_argument("--host_port", type=int, required=True)
    args = parser.parse_args()
    control = HandleKey(host_ip=args.host_ip, port=args.host_port)
    control.run()

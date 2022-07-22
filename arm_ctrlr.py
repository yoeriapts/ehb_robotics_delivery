"""
Adeept Robot Arm Controller
"""
import json
import socket
import threading
import math
import Adeept
#import serial

COMMAND_PORT = 9397
INIT_SERVO_ANGLES = True

class RobotArm:
    """
    """
    running: bool = True

    def to_str(self, indent=0):
        s = "{}RobotArm(\n".format(" "*indent)
        s += "{}  running={}\n".format(" "*indent, self.running)
        s += "{})\n".format(" " * indent)
        return s


# Create a global robotarm
robotarm = RobotArm()


# Thread to read incoming cmds over a socket
class ClientThread(threading.Thread):
    def __init__(self, socket):
        super().__init__()
        self.socket = socket

    def run(self):
        DEBUG = True
        global robot_arm

        while robotarm.running:
            # Read line from socket
            line = ''
            while True:
                c = self.socket.recv(1, socket.MSG_WAITALL).decode("ascii")
                if c == '\n':
                    break
                else:
                    line += c

            # Parse JSON
            if DEBUG:
                print("SERVER GOT:", line)
            data = json.loads(line)

            if 'start' in data:
                data = data['start']

                if len(data) == 5:
                    funcname, p1, p2, p3, p4 = data

                    if funcname == 'servo_write':
                        servo_index = p1
                        angle = p2
                        
                        if DEBUG:
                            print(f"Processing 'servo_write {servo_index} {angle}")

                        if 0 <= servo_index < 5:
                            if -90 <= angle <= 90:
                                # Translate -90..90 to 0..180
                                adeept_angle = 90 + angle
                                Adeept.three_function("'servo_write'", servo_index, adeept_angle)
                            else:
                                print(f"servo_write: servo angle {angle} out of range [-90, 90]")
                        else:
                            print(f"servo_write: servo index {servo_index} out of range")


# ServerThread listens to incoming connections and then starts ClientThread to process commands
class ServerThread(threading.Thread):
    def __init__(self):
        super().__init__()

        # socket for command handling over TCP port
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.s.bind(('0.0.0.0', COMMAND_PORT))
        print(f"Listening for incoming connection on port {COMMAND_PORT}")
        self.s.listen(10)

        self.threads = []
        
        # Serial port towards Arduino
        Adeept.com_init('COM3', 115200, 1)
        Adeept.wait_connect()
        # setup mapping of servos to ctrl pins
        Adeept.three_function("'servo_attach'", 0, 9)
        Adeept.three_function("'servo_attach'", 1, 6)
        Adeept.three_function("'servo_attach'", 2, 5)
        Adeept.three_function("'servo_attach'", 3, 3)
        Adeept.three_function("'servo_attach'", 4, 11)
        
        # Optional init of servo angles
        if INIT_SERVO_ANGLES:
            ANGLE = 90 # this is the angle as used by the robot firmware
            Adeept.three_function("'servo_write'", 0, ANGLE)
            Adeept.three_function("'servo_write'", 1, ANGLE)
            Adeept.three_function("'servo_write'", 2, ANGLE)
            Adeept.three_function("'servo_write'", 3, ANGLE)
            Adeept.three_function("'servo_write'", 4, ANGLE)

    def run(self):
        while robotarm.running:
            # Accept a client
            conn, addr = self.s.accept()

            print(f"Incoming connection from {addr}; starting command-processing thread")
            
            # Create and start the command processing thread
            t = ClientThread(conn)
            t.start()

            self.threads.append(t)


if __name__ == "__main__":
    print("Starting...")

    # Start listening for control connections
    server = ServerThread()
    server.start()

    
"""
virtual robot arm.
Based on original code by Dennis Stechelmackers 2022
"""
import pygame
import json
import socket
import threading
import asyncio
import math
from dataclasses import dataclass

VIEWPORT_W = 1024
VIEWPORT_H = 768

@dataclass
class RobotArm:
    """
    """
    target_angles = [math.radians(0.0)] * 3
    current_angles = [math.radians(0.0)] * 3

    arm_lengths = [75, 200, 200] # Each arm has a length
    arm_base = (int(VIEWPORT_W/2), VIEWPORT_H - int(VIEWPORT_H/10)) # Base of the robot-arm
    target_radius = 20

    running: bool = True
    head_x: int = -1
    head_y: int = -1
    target_x: int = -1
    target_y: int = -1

    def to_str(self, indent=0):
        s = "{}RobotArm(\n".format(" "*indent)
        s += "{}  running={}\n".format(" "*indent, self.running)
        s += "{}  head_x={}\n".format(" "*indent, self.head_x)
        s += "{}  head_y{}\n".format(" "*indent, self.head_y)
        s += "{}  target_x={}\n".format(" "*indent, self.target_x)
        s += "{}  target_y{}\n".format(" "*indent, self.target_y)
        s += "{})\n".format(" " * indent)
        return s


# Create a global robotarm
robotarm = RobotArm(target_x = -1, target_y = -1)


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

                        if 0 <= servo_index < len(robotarm.target_angles):
                            if -90 <= angle <= 90:
                                robotarm.target_angles[servo_index] = math.radians(angle) # Go from degrees to radians
                            else:
                                print(f"servo_write: servo angle {angle} out of range (-90, 90)")
                        else:
                            print(f"servo_write: servo index {servo_index} out of range")

                    if funcname == 'get_current':
                        if DEBUG:
                            print("Processing 'get_current'")
                        print(f"x = {robotarm.head_x}, y = {robotarm.head_y}")
                        
                        outdata = dict(x=robotarm.head_x, y=robotarm.head_y)
                        outdata_json_str = json.dumps(outdata) + '\n'
                        outdata_bytes = bytes(outdata_json_str, "utf8")
                        print(f"outdata_bytes = {outdata_bytes}")
                        self.socket.sendall(outdata_bytes)

                    if funcname == 'get_target':
                        if DEBUG:
                            print("Processing 'get_target'")
                            print(f"current target is: x = {robotarm.target_x}, y = {robotarm.target_y}")
    
                        outdata = dict(x=robotarm.target_x, y=robotarm.target_y)
                        outdata_json_str = json.dumps(outdata) + '\n'
                        outdata_bytes = bytes(outdata_json_str, "utf8")
                        print(f"outdata_bytes = {outdata_bytes}")
                        self.socket.sendall(outdata_bytes)

                    if funcname == 'set_target':
                        if DEBUG:
                            print(f"Processing 'set_target {p1}, {p2}'")
                        robotarm.target_x = p1
                        robotarm.target_y = p2
                        
                    if funcname == 'stop':
                        if DEBUG:
                            print("Processing 'stop''")
                        robotarm.running = False


# ServerThread listens to incoming connections and then starts ClientThread to process commands
class ServerThread(threading.Thread):
    def __init__(self):
        super().__init__()

        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.s.bind(('0.0.0.0', 9395))
        print(f"Listening for incoming connection on port {9395}")
        self.s.listen(10)

        self.threads = []

    def run(self):
        while robotarm.running:
            # Accept a client
            conn, addr = self.s.accept()

            print(f"Incoming connection from {addr}; starting command-processing thread")
            
            # Create and start the command processing thread
            t = ClientThread(conn)
            t.start()

            self.threads.append(t)


# Pygame ebents
def double_click(pos):
    print(f"double_click called: evt.pos {pos}")
    robotarm.target_x = pos[0]
    robotarm.target_y = pos[1]


def check_events():
    global double_click_event, timer

    events = pygame.event.get()
    for event in events:
        if event.type == pygame.QUIT:
            exit()

        if event.type == pygame.MOUSEBUTTONDOWN:
            if timer == 0:
                pygame.time.set_timer(double_click_event, 500)
                timerset = True
            else:
                if timer == 1:
                    pygame.time.set_timer(double_click_event, 0)
                    double_click(event.pos) # location
                    timerset = False

            if timerset:
                timer = 1
                return
            else:
                timer = 0
                return

        elif event.type == double_click_event:
            # timer timed out
            pygame.time.set_timer(double_click_event, 0)
            timer = 0


server = ServerThread()
server.start()

# Display the robot arm
pygame.init()
pygame.display.init()
screen = pygame.display.set_mode((VIEWPORT_W, VIEWPORT_H))
clock = pygame.time.Clock()
double_click_event = pygame.USEREVENT + 1
timer = 0

while robotarm.running:
    surf = pygame.Surface(screen.get_size())

    # Clear the screen
    pygame.draw.rect(surf, pygame.Color('white'), surf.get_rect())

    # Draw the robot segments as rotated rectangles
    x, y = robotarm.arm_base

    # Draw the robot arm's base-plate
    pygame.draw.rect(surf, color=pygame.Color('black'), rect=pygame.Rect(x-30, y-10, 60, 20))

    angle = math.radians(0)             # 0 means upwards
    
    for joint_index in range(len(robotarm.target_angles)):
        # Update the current position
        robotarm.current_angles[joint_index] += 1 * (robotarm.target_angles[joint_index] - robotarm.current_angles[joint_index]) # XXX !!!
        
        # Add a green dot at the bottom of the arm (indicates the joint)
        pygame.draw.circle(surf, color=pygame.Color('green'), center=(x, y), radius=5)

        # Compute end coordinates
        new_angle = angle + robotarm.current_angles[joint_index]
        new_y = y - robotarm.arm_lengths[joint_index] * math.cos(new_angle)
        new_x = x - robotarm.arm_lengths[joint_index] * math.sin(new_angle)

        # Make a polygon
        dx = 10 * math.cos(new_angle)
        dy = -10. * math.sin(new_angle)

        points = [
            (x - dx, y - dy),
            (x + dx, y + dy),
            (new_x + dx, new_y + dy),
            (new_x - dx, new_y - dy),
        ]

        pygame.draw.polygon(surf, color=pygame.Color('black'), points=points)

        # Go to the next segment
        x = new_x
        y = new_y
        angle = new_angle

    # Add a red target dot at the tip of the arm
    #pygame.draw.circle(surf, color=pygame.Color('red'), center=(x, y), radius=robotarm.target_radius)
    pygame.draw.circle(surf, color=pygame.Color((211,31,75)), center=(x, y), radius=robotarm.target_radius)

    robotarm.head_x = int(x)
    robotarm.head_y = int(y)

    # Add a blue target cross to represent the target
    if robotarm.target_x != -1 and robotarm.target_y != -1:
        tcolor = pygame.Color('blue')
        tradius = 10
        llength = 15 # half of line length
        lwidth = 3  # line width
        pygame.draw.line(surf, color=tcolor,
                         start_pos=(robotarm.target_x, robotarm.target_y - llength),
                         end_pos=(robotarm.target_x, robotarm.target_y + llength), width=lwidth)
        pygame.draw.line(surf, color=tcolor,
                         start_pos=(robotarm.target_x - llength, robotarm.target_y),
                         end_pos=(robotarm.target_x + llength, robotarm.target_y), width=lwidth)

    # Display
    clock.tick(60)
    check_events()
    screen.blit(surf, (0, 0))
    pygame.event.pump()
    pygame.display.flip()
    
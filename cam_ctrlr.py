import numpy as np
import cv2
import socket, threading, json
from cv2_utils import find_vidcapture

COMMAND_PORT = 9396
DEBUG = False


INSTRUCTIONS = \
"""
  Type Q to quit
  Type R to report the coordinates
  Double click on image to set target
"""


def process_image(image):
    """
    :param image:
    :return: mask_red, mask_red_morphed, centroid_x, centroid_y
    """
    mask_red = (image_hsv[:, :, 1] > 120) & (image_hsv[:, :, 0] > 150)

    # Create a structuring element for OpenCV morphological operations
    element = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

    # perform dilations and erosions on the mask obtain a nice object without gaps, and without background noise
    mask_red = mask_red.astype(np.uint8)  # convert type

    mask_red_morphed = cv2.dilate(mask_red, element)
    mask_red_morphed = cv2.erode(mask_red_morphed, element, iterations=2)
    # mask2 = cv2.erode(mask2, element)
    mask_red_morphed = cv2.dilate(mask_red_morphed, element)
    mask_red_morphed = cv2.erode(mask_red_morphed, element, iterations=2)
    # mask2 = cv2.erode(mask2, element)
    mask_red_morphed = cv2.dilate(mask_red_morphed, element)

    # find centroids
    retval, labels, stats, centroids = cv2.connectedComponentsWithStats(mask_red_morphed)
    # print(retval, stats, centroids)
    
    if retval <= 1:
        # No centroid found
        cx, cy = -1, -1
    else:
        # find largest one
        max_i = np.argmax(stats[1:, 4]) + 1
        if DEBUG:
            max_area = stats[max_i, 4]
            print("The largest component is", max_i, "of area", max_area, "and has centroid", centroids[max_i])
        cx, cy = centroids[max_i]

    return mask_red, mask_red_morphed, cx, cy


def add_markers_to_image(image_bgr, head=(-1, -1), target=(-1, -1)):
    """
    Add markers for head and target to a copy of the image. Original image is not changed
    :param image_bgr: 3D np array
    :param head: (x,y) of head
    :param target: (x, y) of target
    :return: 3D np array with markers
    """
    image_markers = np.copy(image_bgr)
    height, width, _ = image_markers.shape

    head_color = (255, 255, 255)
    cx = head[0]; cy = head[1]
    hl = 25  # half lenght of cross
    ht = 3  # half thickness of cross
    image_markers[max(int(cy-hl), 0):min(int(cy+hl), height-1), max(int(cx-ht), 0):min(int(cx+ht), width-1), :] = head_color
    image_markers[max(int(cy-ht), 0):min(int(cy+ht), height-1), max(int(cx-hl), 0):min(int(cx+hl), width-1), :] = head_color

    target_color = (255, 0, 0)
    cx = target[0]; cy = target[1]
    hl = 20
    ht = 2
    image_markers[max(int(cy-hl), 0):min(int(cy+hl), height-1), max(int(cx-ht), 0):min(int(cx+ht), width-1), :] = target_color
    image_markers[max(int(cy-ht), 0):min(int(cy+ht), height-1), max(int(cx-hl), 0):min(int(cx+hl), width-1), :] = target_color

    return image_markers


def mouse_callback(event, x, y, flags, param):
    # mouse callback function
    if event == cv2.EVENT_LBUTTONDBLCLK:
        print(f"Double clicked on ({x},{y})")
        global target_x, target_y
        target_x = x
        target_y = y


# Thread to read incoming cmds over a socket
class ClientThread(threading.Thread):
    def __init__(self, socket):
        super().__init__()
        self.socket = socket
    
    def run(self):
        DEBUG = True
        global target_x, target_y

        while True:
    
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
                    
                    if funcname == 'get_current':
                        if DEBUG:
                            print("Processing 'get_current'")
                        print(f"x = {head_x}, y = {head_y}")

                        outdata = dict(x=head_x, y=head_y)
                        outdata_json_str = json.dumps(outdata) + '\n'
                        outdata_bytes = bytes(outdata_json_str, "utf8")
                        print(f"outdata_bytes = {outdata_bytes}")
                        self.socket.sendall(outdata_bytes)
                    
                    if funcname == 'get_target':
                        if DEBUG:
                            print("Processing 'get_target'")
                            print(f"current target is: x = {target_x}, y = {target_y}")
                        outdata = dict(x=target_x, y=target_y)
                        outdata_json_str = json.dumps(outdata) + '\n'
                        outdata_bytes = bytes(outdata_json_str, "utf8")
                        print(f"outdata_bytes = {outdata_bytes}")
                        self.socket.sendall(outdata_bytes)
                    
                    if funcname == 'set_target':
                        if DEBUG:
                            print(f"Processing 'set_target {p1}, {p2}'")
                        target_x = p1
                        target_y = p2


# ServerThread listens to incoming connections and then starts ClientThread to process commands
class ServerThread(threading.Thread):
    def __init__(self):
        super().__init__()
        
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.s.bind(('0.0.0.0', COMMAND_PORT))
        self.s.listen(10)
        print(f"Listening for incoming connection on port {COMMAND_PORT}")
        self.threads = []
    
    def run(self):
        while True:
            # Accept a client
            conn, addr = self.s.accept()
            
            print(f"Incoming connection from {addr}; starting command-processing thread")
            
            # Create and start the command processing thread
            t = ClientThread(conn)
            t.start()
            
            self.threads.append(t)


# globals
target_x = -1
target_y = -1
head_x = -1
head_y = -1

if __name__ == "__main__":
    print("Starting...")

    # Find the right camera
    #cv2.VideoCapture(0, cv2.CAP_DSHOW)
    # find cam w specific resolution (only way to identify cam in set of multiple cams)
    videocap, _ = find_vidcapture(1280, 960)
    if videocap:
        print("Found camera")
    else:
        print("Error! Camera not found... exiting")
    # Set actual resolution to use
    videocap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    videocap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    # Start listening for control connections
    server = ServerThread()
    server.start()

    print(f"Instructions:{INSTRUCTIONS}")
    
    print("Processing images...")
    iteration_counter = 0

    cv2.namedWindow('Camera Image')
    cv2.setMouseCallback('Camera Image', mouse_callback)

    while True:
        _, frame = videocap.read()
        if DEBUG:
            cv2.imshow('Original Camera Image', frame)

        # wait 1 ms, if arg is 0, we wait forever :-(
        keycode = cv2.waitKey(1)
        if keycode & 0xFF == ord('q'):
            break
            
        if keycode & 0xFF == ord('r') or iteration_counter % 100 == 0:
            print(f"Iteration {iteration_counter}: center = ({head_x}, {head_y}), target = ({target_x}, {target_y})")
    
        image_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
       
        image_red, image_red_morphed, head_x, head_y = process_image(image_hsv)
        image_red = image_red * 128
        image_red_morphed = image_red_morphed * 128
        
        if DEBUG:
            cv2.imshow('Red', image_red)
            cv2.imshow('Red Morphed', image_red_morphed)
        
        frame_markers = add_markers_to_image(frame, head=(head_x, head_y), target=(target_x, target_y))
        
        # show camera frame with markers
        cv2.imshow('Camera Image', frame_markers)

        iteration_counter += 1
        
    # closing all open windows
    cv2.destroyAllWindows()
    print("done")

"""
Sone utility functions for working with open cv v2
"""

"""
Find Video Capture Devices (Cameras) and their max resolutions
"""

import cv2

def get_maxres_videocapture(id):
    """
    Get the max resolution of the device with 'id'
    :param id: id of the video capture device
    :return: (None, 0, 0) or (VideoCaptureDevice, maxwidth, maxheight)
    """
    
    cap = cv2.VideoCapture(id, cv2.CAP_DSHOW)
    #cap = cv2.VideoCapture(source, cv2.CAP_MSMF)
    if cap is None or not cap.isOpened():
        #print(f"Warning: unable to open video source: {source}")
        return None, 0, 0
    else:
        #print(f"Found capture device {cap} at source {source}")
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 20000)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 20000)
        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        return cap, width, height


def find_vidcapture(width, height):
    """
    Find the first video capture device with max resolution (width, height)
    
    :param width: width
    :param height: height
    :return: tuple (the open cv2.VideoCaptureDevice device, its id) when found or (None, None) if no cam found
    """
    
    DEBUG = False
    
    if DEBUG: print(f"Looking for Video Capture Device with max resolution {width} x {height}")
    
    for id in range(10):
        cap, w, h = get_maxres_videocapture(id)
        if DEBUG: print(f"Found a device with id:{id}, cap:{cap}, max width:{w}, max height:{h}")
        if cap and width == w and height == h:
            if DEBUG: print(f"Found match, returning")
            return cap, id
        if cap:
            cap.release()

    return None, None

    
if __name__ == "__main__":
    
    def test_find_dev_w_maxres(w, h):
        print(f"Find a device with max resolution {w}x{h}")
        cap, id = find_vidcapture(w, h)
        if cap:
            print(f"Found: {cap} with id {id}")
            cap.release()
        else:
            print("Not found")

    test_find_dev_w_maxres(1280, 960)
    test_find_dev_w_maxres(640, 480)
    test_find_dev_w_maxres(320,240)
    
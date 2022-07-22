"""
rac: Robot Arm and Camera client

Command line utility for testing the (virtual) robot arm controller and camera controller
Use -h or --help to get the commands help
"""

import os, time, sys
import argparse
import socket
import json


def args_to_report_str(args, headline: str):
    d = vars(args)
    s = "{}".format(headline)
    for k in d:
        s += "\n  {}: {}".format(k, d[k])
    return s


CMD_LINE_HELP_EPILOG = """
examples:
  rac -i servo-index -a servo-angle  // set servo to angle
  rac -x target-x -y target-y        // set target coordinates to (target-x, target-y)
  rac -c                             // get current head coordinates
  rac -t                             // get current target coordinates
"""
DEFAULT_HOST="localhost"
DEFAULT_PORT="9395"

def parse_args():
    """
    Set up the command arg parser
    :return: Namespace object with all the args
    """
    parser = argparse.ArgumentParser(description="Robot-Arm and Camera client",
                                     formatter_class=argparse.RawTextHelpFormatter,
                                     epilog=CMD_LINE_HELP_EPILOG)

    parser.add_argument('-o', '--host', type=str,
                        help="target hostname or ip-address",
                        required=False, default=DEFAULT_HOST)
    parser.add_argument('-p', '--port', type=int,
                        help="target port",
                        required=False, default=DEFAULT_PORT)
    parser.add_argument('-i', '--si', type=int,
                        help="index of servo",
                        required=False, default=None)
    parser.add_argument('-a', '--angle', type=int,
                        help="angle of servo speicified with -i",
                        required=False, default=None)
    parser.add_argument('-x', '--tx', type=int,
                        help="set target's x-coordinate",
                        required=False, default=None)  # 'None' means not defined
    parser.add_argument('-y', '--ty', type=int,
                        help="set target's y-coordinate",
                        required=False, default=None)  # 'None' means not defined
    parser.add_argument('-t', '--gt', action="store_true",
                        help="get current location of target",
                        required=False, default=False)
    parser.add_argument('-c', '--gc', action="store_true",
                        help="get current location of arm's head",
                        required=False, default=False)
    parser.add_argument('-s', '--stop', action="store_true",
                        help="stop the server.",
                        required=False, default=False)

    return parser.parse_args()


def send_json_cmd(s, cmd):
    """
    Send json formatted command over socket
    :param s: the socket
    :param cmd: a dict containing json
    :return: nothing
    """
    #print(f"send_json_cmd: cmd = {cmd}")
    cmd_str = json.dumps(cmd) + '\n'
    cmd_bytes = bytes(cmd_str, 'utf8')
    # print(command_bytes)
    s.sendall(cmd_bytes)


def rcv_json(s):
    """
    Receive a dict (from a json formatted string) from a socket
    :param s: the socket
    :return: dict
    """

    # Read line from socket
    line = ''
    while True:
        c = s.recv(1, socket.MSG_WAITALL).decode("ascii")
        if c == '\n':
            break
        else:
            line += c
    #print(f"rcv_json: line = {line}")
    return json.loads(line)


DEBUG = False
def dprint(*args, **kwargs):
    if DEBUG:
        print(*args, **kwargs)


def main():
    args = parse_args()
    # If you don't want to use command line arguments:
    #   comment out the line above and
    #   uncomment the ones below to fill in the 'args' object manually

    # class Args:
    #     si = 2
    #     angle = 70
    # ...
    # args = Args()
    # args.tx = 123

    dprint(args_to_report_str(args, "args:"))
    arg_error = False

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((args.host, args.port))
    
    if args.si is not None:
        if args.angle is not None:
            command = {'start': ['servo_write', args.si, args.angle, 0, 0]}
            send_json_cmd(s, command)
        else:
            print("Should specify angle when specifying servo-index")
            arg_error = True

    if args.tx is not None and args.ty is not None:
        command = {'start': ['set_target', args.tx, args.ty, 0, 0]}
        send_json_cmd(s, command)

    if args.gc:
        command = {'start': ['get_current', 0, 0, 0, 0]}    # add 4 useless params
        send_json_cmd(s, command)
        data = rcv_json(s)
        print(f"data = {data}, type = {type(data)}")
        print(data['x'], data['y'])

    if args.gt:
        command = {'start': ['get_target', 0, 0, 0, 0]}    # add 4 useless params
        send_json_cmd(s, command)
        data = rcv_json(s)
        print(f"data = {data}, type = {type(data)}")
        print(data['x'], data['y'])

    if args.stop:
        command = {'start': ['stop', 0, 0, 0, 0]}    # add 4 useless params
        send_json_cmd(s, command)
        
    if arg_error:
        print("Error in command line arguments")
        
    s.close()


if __name__ == '__main__':
    main()

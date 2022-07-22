import math
from time import sleep
import socket
import numpy as np
from random import randint

import gym
import stable_baselines3 as sb3
import stable_baselines3.common.env_checker

from rac import send_json_cmd, rcv_json
from gym_myenv import MyEnv_wTarget

if __name__ == "__main__":
    
    robotarm_host = 'localhost'
    robotarm_port = 9395
    cam_host = 'localhost'
    cam_port = 9395
    viewport = (1024, 768)
    target = (-1, -1) # XXX must be valid
    arm_cam_delay = 0.025
    use_servo_0 = True

    # Connect to Arm and Camera
    arm_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    arm_sock.connect((robotarm_host, robotarm_port))

    if robotarm_host == cam_host and robotarm_port == cam_port:
        cam_sock = arm_sock  # use same socket
    else:
        # open cam socket
        cam_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        cam_sock.connect((cam_host, cam_port))
    print(f"Connected to Arm and Camera")

    # load model
    agent = sb3.PPO.load("./saved_models/PPO_MyEnv_wTarget_99.zip")
    print(f"Loaded agent model")

    while True:
        # Set target in Camera window
        target_x = target[0]; target_y = target[1]
        cmd = {'start': ['set_target', target_x, target_y, 0, 0]}
        send_json_cmd(cam_sock, cmd)
        print(f"Waiting for the target location")
    
        counter = 0
        while True:
            if counter % 30 == 0:
                print(f"counter = {counter}: waiting for target location")
            # Get target location, if updagted by theu ser it should be different from -1,-1
            command = {'start': ['get_target', 0, 0, 0, 0]}  # add 4 useless params
            send_json_cmd(cam_sock, command)
            data = rcv_json(cam_sock)
            target_x = data['x']
            target_y = data['y']
            if target_x != -1 and target_y != -1:
                break
            sleep(0.1)
            counter += 1

        target = (target_x, target_y)
        print(f"Got new target location: {target}")
        
        # create env
        myenv = MyEnv_wTarget(viewport, target, arm_sock, cam_sock, arm_cam_delay, use_servo_0)
        # run preliminary check
        stable_baselines3.common.env_checker.check_env(myenv)

        # Set env to agent
        state = myenv.reset()
        agent.set_env(myenv)
        for step in range(5120):
            action, _ = agent.predict(state)
            next_state, reward, done, info = myenv.step(action)

            print(f"step #{step}: next_state={next_state}, reward={reward}, done={done}, info={info}")
            state = next_state
            
            if done:
                break
            
            # env.render()

        #print(f"step #{step}: reward={reward}, done={done}, info={info}")
        break
    
    
    

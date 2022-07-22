import math
from time import sleep
import socket
import numpy as np
from random import randint

import gym
import stable_baselines3 as sb3

from rac import send_json_cmd, rcv_json


class MyEnv(gym.Env):
    def __init__(self, viewport, target, arm_socket, cam_socket, arm_cam_delay=0.075, use_servo_0=True):
        """
        :param target_x: x coord of target position to reach
        :param target_y: y coord of target position to reach
        :param arm_socket: socket to connected to the robot arm (Note: socket must already be connected!)
        :param cam_socket: socket connected to the camera controller (Note: socket must already be connected!)
        :param arm_cam_delay: delay between moving robot arm and asking cam for new head position (can be lower on simulations)
        :param use_servo_0: If False this servo will not be moved, useful for the physical Adeept Arm
        """
        super().__init__()
        
        self.episode_max_steps = 512
        
        # Remember sizes of viewport, depends on resolution of camera
        self.viewport_w = viewport[0]
        self.viewport_h = viewport[1]
        
        # Remember target location, this location does not change during lifetime of an MyEnv
        self.target_x = target[0]
        self.target_y = target[1]
        
        # socket already connected to Robot Arm controller
        self.arm_socket = arm_socket
        
        # socket already connected to the Camera controller
        self.cam_socket = cam_socket
        
        # delay between moving robot arm and asking cam for new head position
        self.arm_cam_delay = arm_cam_delay
        
        # Set target in Camera window
        cmd = {'start': ['set_target', self.target_x, self.target_y, 0, 0]}
        send_json_cmd(self.cam_socket, cmd)

        if use_servo_0:
            self.ACTION_TO_DELTA_ANGLES = [
                (-1, 0, 0),
                (+1, 0, 0),
                (0, -1, 0),
                (0, +1, 0),
                (0, 0, -1),
                (0, 0, +1),
                (-2, 0, 0),
                (+2, 0, 0),
                (0, -2, 0),
                (0, +2, 0),
                (0, 0, -2),
                (0, 0, +2),
                (-4, 0, 0),
                (+4, 0, 0),
                (0, -4, 0),
                (0, +4, 0),
                (0, 0, -4),
                (0, 0, +4),
                (0, -8, 0),
                (0, +8, 0),
                (0, 0, -8),
                (0, 0, +8),
                (0, 0, -16),
                (0, 0, +16)
            ]
        else:
            # servo 0 never moves
            self.ACTION_TO_DELTA_ANGLES = [
                (0, -1, 0),
                (0, +1, 0),
                (0, 0, -1),
                (0, 0, +1),
                (0, -2, 0),
                (0, +2, 0),
                (0, 0, -2),
                (0, 0, +2),
                (0, -4, 0),
                (0, +4, 0),
                (0, 0, -4),
                (0, 0, +4),
                (0, -8, 0),
                (0, +8, 0),
                (0, 0, -8),
                (0, 0, +8),
                (0, 0, -16),
                (0, 0, +16)
            ]
            
        self.action_space = gym.spaces.Discrete(len(self.ACTION_TO_DELTA_ANGLES))
        self.observation_space = gym.spaces.Box(low=np.array([0, 0]),
                                                high=np.array([self.viewport_w - 1, self.viewport_h - 1]),
                                                dtype=np.float32)
        
        self.cum_reward = 0
        self.step_counter = 0
        self.head_out_of_view_cntr = 0  # Count how many times head is not seen
        
        # reset
        self.reset()
    
    def _apply_angles(self, angles, arm_cam_delay=None):
        """
        send new angles to robot, return new (x, y) of head
        """
        for i, a in enumerate(angles):
            cmd = {'start': ['servo_write', i, int(a), 0,
                             0]}  # type(a) is numpy.int32, must be converted to int for json serialisation
            # print(f"_apply_angles: {cmd}")
            send_json_cmd(self.arm_socket, cmd)
        
        if arm_cam_delay is None:
            sleep(self.arm_cam_delay)
        else:
            sleep(arm_cam_delay)
        
        # Get current head location from the camera
        command = {'start': ['get_current', 0, 0, 0, 0]}  # add 4 useless params
        send_json_cmd(self.cam_socket, command)
        data = rcv_json(self.cam_socket)
        
        # print(f"data = {data}, type = {type(data)}")
        # print(data['x'], data['y'])
        return data['x'], data['y']
    
    def _reset_robotarm(self):
        self.angles = np.array([0, 0, 0])  # all engines start at angles 0
        # Apply angles and read new x,y of head, wait a full second after setting the head to its initial position
        self.current_x, self.current_y = self._apply_angles(self.angles, arm_cam_delay=1)
    
    def reset(self):
        self.cum_reward = 0
        self.step_counter = 0
        self.head_out_of_view_cntr = 0
        
        self._reset_robotarm()
        self.min_distance = math.dist((self.target_x, self.target_y), (self.current_x, self.current_y))
        print(f"MyEnv: reset: min_distance = {self.min_distance}")
        # return np.array([self.current_x, self.current_y, self.target_x, self.target_y])
        return np.array([self.current_x, self.current_y])
    
    def step(self, a):
        # check for valid action (should never happen though !)
        if a < 0 or a >= len(self.ACTION_TO_DELTA_ANGLES):
            return None  # Maybe raise some exception ? XXX
        
        # Keep track of number of steps
        self.step_counter += 1
        
        # Initialise return variables
        reward = 0
        dist_current = 0
        done = False
        info = {}
        
        # Calculate new step-motor angles; check bounds
        angles_new = self.angles + self.ACTION_TO_DELTA_ANGLES[a]
        
        if np.any(angles_new < [-90, -90, -90]) or np.any(angles_new > [90, 90, 90]):
            # Illegal angles reached -> return without changing robot arm
            reward = 0
            done = False
            info = {'error': 'angles out of bounds'}
        
        else:
            # Apply new angles, get new coordinates of head
            new_x, new_y = self._apply_angles(angles_new)
            
            if (new_x < 0) or (new_x > self.viewport_w - 1) or (new_y < 0) or (new_y > self.viewport_h - 1):
                def clamp(n, smallest, largest):
                    return max(smallest, min(n, largest))
                
                # reapply existing angles
                self.current_x, self.current_y = self._apply_angles(self.angles)
                # x,y should ideally be the same as before, but clamp anyway
                self.current_y = clamp(self.current_y, 0, self.viewport_h - 1)
                self.current_x = clamp(self.current_x, 0, self.viewport_w - 1)
                
                reward = 0
                info = {'error': f"arm's head out of view (cntr = {self.head_out_of_view_cntr}"}
                self.head_out_of_view_cntr += 1
                if self.head_out_of_view_cntr >= 10:
                    # Haven't detected head for 10 steps, something is wrong !
                    # Force the end of this episode
                    info = {'error': "arm's head out of view for to many times"}
                    done = True
            
            else:
                # apply angles and head position
                prev_x, prev_y = self.current_x, self.current_y
                self.angles = angles_new
                self.current_x, self.current_y = new_x, new_y
                
                dist_previous = math.dist((prev_x, prev_y), (self.target_x, self.target_y))
                dist_current = math.dist((self.current_x, self.current_y), (self.target_x, self.target_y))
                
                # Are we done? Goal reached or max nmbr of steps reached
                done = False
                info = {}
                
                if dist_current < 10:
                    reward = 0
                    for r in range(int(self.min_distance), 0, -1):
                        reward += 1 / r
                    self.min_distance = max(1, dist_current)  # never go to 0
                    
                    # Add extra reward for less steps
                    reward += 5 * (1 - (self.step_counter / self.episode_max_steps))
                    
                    done = True
                    info = {'done': 'head reached target'}
                
                else:
                    if dist_current < self.min_distance:
                        # collect all sub-rewards for passing the min dist
                        reward = 0
                        for r in range(int(self.min_distance), int(dist_current), -1):
                            reward += 1 / r
                        self.min_distance = dist_current
                    else:
                        reward = 0
                
                if self.step_counter >= self.episode_max_steps:
                    done = True
                    reward = 0
                    info = {'done': 'max step count reached'}
        
        self.cum_reward += reward
        
        if self.step_counter % 50 == 0 or reward != 0 or done or info:
            print(
                f"step #{self.step_counter}: dist_current={dist_current}, done={done}, reward={reward}, cum_reward={self.cum_reward}, info={info}")
        
        # return np.array([self.current_x, self.current_y, self.target_x, self.target_y]), reward, done, info
        return np.array([self.current_x, self.current_y]), reward, done, info


class MyEnv_wTarget(gym.Env):
    def __init__(self, viewport, target, arm_socket, cam_socket, arm_cam_delay=0.075, use_servo_0=True):
        """
        :param target_x: x coord of target position to reach
        :param target_y: y coord of target position to reach
        :param arm_socket: socket to connected to the robot arm (Note: socket must already be connected!)
        :param cam_socket: socket connected to the camera controller (Note: socket must already be connected!)
        :param arm_cam_delay: delay between moving robot arm and asking cam for new head position (can be lower on simulations)
        :param use_servo_0: If False this servo will not be moved, useful for the physical Adeept Arm
        """
        super().__init__()

        self.episode_max_steps = 512
        self.min_distance = 0
        
        # Remember sizes of viewport, depends on resolution of camera
        self.viewport_w = viewport[0]
        self.viewport_h = viewport[1]

        # Remember target location, this location does not change during lifetime of an MyEnv
        self.target_x = target[0]
        self.target_y = target[1]
        
        # socket already connected to Robot Arm controller
        self.arm_socket = arm_socket
        
        # socket already connected to the Camera controller
        self.cam_socket = cam_socket
        
        # delay between moving robot arm and asking cam for new head position
        self.arm_cam_delay = arm_cam_delay
        
        # Set target in Camera window
        cmd = {'start': ['set_target', self.target_x, self.target_y, 0, 0]}
        send_json_cmd(self.cam_socket, cmd)

        if use_servo_0:
            self.ACTION_TO_DELTA_ANGLES = [
                (-1, 0, 0),
                (+1, 0, 0),
                (0, -1, 0),
                (0, +1, 0),
                (0, 0, -1),
                (0, 0, +1),
                (-2, 0, 0),
                (+2, 0, 0),
                (0, -2, 0),
                (0, +2, 0),
                (0, 0, -2),
                (0, 0, +2),
                (-4, 0, 0),
                (+4, 0, 0),
                (0, -4, 0),
                (0, +4, 0),
                (0, 0, -4),
                (0, 0, +4),
                (0, -8, 0),
                (0, +8, 0),
                (0, 0, -8),
                (0, 0, +8),
                (0, 0, -16),
                (0, 0, +16)
            ]
        else:
            # servo 0 never moves
            self.ACTION_TO_DELTA_ANGLES = [
                (0, -1, 0),
                (0, +1, 0),
                (0, 0, -1),
                (0, 0, +1),
                (0, -2, 0),
                (0, +2, 0),
                (0, 0, -2),
                (0, 0, +2),
                (0, -4, 0),
                (0, +4, 0),
                (0, 0, -4),
                (0, 0, +4),
                (0, -8, 0),
                (0, +8, 0),
                (0, 0, -8),
                (0, 0, +8),
                (0, 0, -16),
                (0, 0, +16)
            ]

        self.action_space = gym.spaces.Discrete(len(self.ACTION_TO_DELTA_ANGLES))
        # self.observation_space = gym.spaces.Box(low=np.array([0, 0]),
        #                                         high=np.array([self.viewport_w - 1, self.viewport_h - 1]),
        #                                         dtype=np.float32)
        self.observation_space = gym.spaces.Box(low=np.array([0, 0, 0, 0]),
                                                high=np.array([self.viewport_w - 1, self.viewport_h - 1,
                                                               self.viewport_w - 1, self.viewport_h - 1]),
                                                dtype=np.float32)

        self.cum_reward = 0
        self.step_counter = 0
        self.head_out_of_view_cntr = 0   # Count how many times head is not seen
        
        # reset
        self.reset()
    
    def _apply_angles(self, angles, arm_cam_delay = None):
        """
        send new angles to robot, return new (x, y) of head
        """
        for i, a in enumerate(angles):
            cmd = {'start': ['servo_write', i, int(a), 0,
                             0]}  # type(a) is numpy.int32, must be converted to int for json serialisation
            # print(f"_apply_angles: {cmd}")
            send_json_cmd(self.arm_socket, cmd)
        
        if arm_cam_delay is None:
            sleep(self.arm_cam_delay)
        else:
            sleep(arm_cam_delay)
        
        # Get target location (in case this was updated by the user)
        command = {'start': ['get_target', 0, 0, 0, 0]}  # add 4 useless params
        send_json_cmd(self.cam_socket, command)
        data = rcv_json(self.cam_socket)
        target_x = data['x']
        target_y = data['y']

        # Get current head location from the camera
        command = {'start': ['get_current', 0, 0, 0, 0]}  # add 4 useless params
        send_json_cmd(self.cam_socket, command)
        data = rcv_json(self.cam_socket)
        # print(f"data = {data}, type = {type(data)}")
        # print(data['x'], data['y'])
        current_x = data['x']
        current_y = data['y']
        return current_x, current_y, target_x, target_y
    
    def _reset_robotarm(self):
        self.angles = np.array([0, 0, 0])  # all engines start at angles 0
        # Apply angles and read new x,y of head, wait a full second after setting the head to its initial position
        self.current_x, self.current_y, _, _ = self._apply_angles(self.angles, arm_cam_delay=1)
    
    def reset(self):
        print("MyEnv_wTarget: reset")
        self.cum_reward = 0
        self.step_counter = 0
        self.head_out_of_view_cntr = 0
        
        self._reset_robotarm()
        
        self.min_distance = math.dist((self.target_x, self.target_y), (self.current_x, self.current_y))
        print(f"MyEnv_wTarget: reset: min_distance = {self.min_distance}")
        return np.array([self.current_x, self.current_y, self.target_x, self.target_y])
        #return np.array([self.current_x, self.current_y])

    def close(self):
        super().close()
        print("MyEnv_wTarget: close")

    def set_target(self, target):
        self.target_x,  self.target_y = target
        print(f"MyEnv_wTarget: set_target, target = ({self.target_x}, {self.target_y})")
        # Set target in Camera window
        cmd = {'start': ['set_target', self.target_x, self.target_y, 0, 0]}
        send_json_cmd(self.cam_socket, cmd)

    def step(self, a):
        # check for valid action (should never happen though !)
        if a < 0 or a >= len(self.ACTION_TO_DELTA_ANGLES):
            return None  # Maybe raise some exception ? XXX
        
        # Keep track of number of steps
        self.step_counter +=1

        # Initialise return variables
        reward = 0
        dist_current = 0
        done = False
        info = {}

        # Calculate new step-motor angles; check bounds
        angles_new = self.angles + self.ACTION_TO_DELTA_ANGLES[a]
        
        if np.any(angles_new < [-90, -90, -90]) or np.any(angles_new > [90, 90, 90]):
            # Illegal angles reached -> return without changing robot arm
            reward = 0
            done = False
            info = dict(target_reached=False, msg='angles out of bounds')
            
        else:
            # Apply new angles, get new coordinates of head and target
            new_x, new_y, target_x, target_y = self._apply_angles(angles_new)

            # if the new target position is different from the existing one, the episode should end
            if (target_x, target_y) != (self.target_x, self.target_y):
                # Store new target
                self.target_x, self.target_y = target_x, target_y
                reward = 0
                done = True
                info = dict(target_reached=False, msg='target location changed')

            # Has the head moved out of bounds? Handle it!
            elif (new_x < 0) or (new_x > self.viewport_w - 1) or (new_y < 0) or (new_y > self.viewport_h - 1):
                def clamp(n, smallest, largest):
                    return max(smallest, min(n, largest))
                
                # reapply existing angles
                self.current_x, self.current_y, _, _ = self._apply_angles(self.angles)
                # x,y should ideally be the same as before, but clamp anyway
                self.current_y = clamp(self.current_y, 0, self.viewport_h - 1)
                self.current_x = clamp(self.current_x, 0, self.viewport_w - 1)
                
                reward = 0
                info = dict(target_reached=False, msg=f"arm's head out of view (cntr = {self.head_out_of_view_cntr}")
                self.head_out_of_view_cntr +=1
                if self.head_out_of_view_cntr >= 10:
                    # Haven't detected head for 10 steps, something is wrong !
                    # Force the end of this episode
                    info = dict(target_reached=False, msg="arm's head out of view for to many times")
                    done = True

            else:
                # apply angles and head position
                prev_x, prev_y = self.current_x, self.current_y
                self.angles = angles_new
                self.current_x, self.current_y = new_x, new_y
                self.target_x, self.target_y = target_x, target_y
                
                dist_previous = math.dist((prev_x, prev_y), (self.target_x, self.target_y))
                dist_current = math.dist((self.current_x, self.current_y), (self.target_x, self.target_y))
                
                # Are we done? Goal reached or max nmbr of steps reached
                done = False
                info = {}
                
                if dist_current < 10:
                    reward = 0
                    for r in range(int(self.min_distance), 0, -1):
                        reward += 1 / r
                    self.min_distance = max(1.0, dist_current)    # never go to 0
                    
                    # Add extra reward for less steps
                    reward += 5*(1-(self.step_counter/self.episode_max_steps))
                    done = True
                    info = dict(target_reached=True, msg="head reached target")
                    
                else:
                    if dist_current < self.min_distance:
                        # collect all sub-rewards for passing the min dist
                        reward = 0
                        for r in range(int(self.min_distance), int(dist_current), -1):
                            reward += 1 / r
                        self.min_distance = dist_current
                    else:
                        reward = 0
        
                if self.step_counter >= self.episode_max_steps:
                    done = True
                    reward = 0
                    info = dict(target_reached=False, msg="max step count reached")
                
        self.cum_reward += reward
        
        #print(f"current = ({self.current_x},{self.current_y}), target = ({self.target_x}, {self.target_y})")
        if self.step_counter % 50 == 0 or reward != 0 or done or info:
            print(f"step #{self.step_counter}: dist_current={dist_current}, min_dist={self.min_distance}, "
                  f"reward={reward}, cum_reward={self.cum_reward}, done={done}, info={info}")
        
        return np.array([self.current_x, self.current_y, self.target_x, self.target_y]), reward, done, info
        #return np.array([self.current_x, self.current_y]), reward, done, info
        

if __name__ == "__main__":
    
    robotarm_host = None
    robotarm_port = None
    cam_host = None
    cam_port = None
    viewport = None
    target = None
    arm_cam_delay = None
    use_servo_0 = None
    
    # Select a setup:
    # 1: local virtual arm and virtual camera
    # 2: remote virtual arm and local real camera
    # 3: remote real robot arm and local real camera
    setup = 3
    if setup == 1:
        # connect to Virtual Robot Arm, which is also used as virtual cam
        robotarm_host = 'localhost'
        robotarm_port = 9395
        cam_host = 'localhost'
        cam_port = 9395
        viewport = (1024, 768)
        target = (300, 400)
        arm_cam_delay = 0.025
        use_servo_0 = True
    elif setup == 2:
        # connect to a Virtual Robot Arm on a remote host and use the cam_trlr
        robotarm_host = "192.168.2.5"
        robotarm_port = 9395
        cam_host = 'localhost'
        cam_port = 9396
        viewport = (640, 480)
        target = (150, 250)
        arm_cam_delay = 0.075
        use_servo_0 = True
    elif setup == 3:
        # connect to the arm_ctrlr, which controlls the Adeept Arm and and use the cam_trlr
        robotarm_host = "localhost"
        robotarm_port = 9397
        cam_host = 'localhost'
        cam_port = 9396
        viewport = (640, 480)
        target = (130, 260)
        arm_cam_delay = 0.1     # Real robot ==> wait longer
        use_servo_0 = False     # Real robot ==> do not use servo 0 (foot)
    else:
        print(f"setup '{setup}' unknown!")
        exit(1)

    # Connect to Arm
    arm_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    arm_sock.connect((robotarm_host, robotarm_port))
    
    if robotarm_host == cam_host and robotarm_port == cam_port:
       cam_sock = arm_sock # use same socket
    else:
        # open cam socket
        cam_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        cam_sock.connect((cam_host, cam_port))

    # Exercise: Use stable_baselines3.common.env_checker.check_env to check your environment
    # NOTE: If check_env raises no exception and prints nothing, your environment is good :-) .
    import stable_baselines3.common.env_checker
    
    #myenv = MyEnv(viewport, target, arm_sock, cam_sock, arm_cam_delay)
    myenv = MyEnv_wTarget(viewport, target, arm_sock, cam_sock, arm_cam_delay, use_servo_0)
    stable_baselines3.common.env_checker.check_env(myenv)
    
    print(f"action_space: {myenv.action_space}")
    print(f"observation_space: {myenv.observation_space}")
    print(f"start state: {myenv.reset()}")
    
    trainingtype = 1
    if trainingtype == 1:
        # Create a sb3.PPO agent for it, and train the agent for 500 time-steps
        print("Train in 1 single run, on 1 target")
        print("*** Learning starts ***")
        #agent = sb3.SAC( # Requires Box as action space
        # There is no PPO2 in my version of stable baselines !?
        agent = sb3.PPO(
            policy="MlpPolicy",
            env=myenv,
            verbose=2, # 2 = debug
            tensorboard_log="./logdir/",
            #learning_rate=0.01,
            #gamma=0.9
            n_steps=512,
            batch_size=16 # 64 is default
        )
        
        iterations = 20
        timesteps = 512 * iterations
        model = agent.learn(total_timesteps=timesteps)

    elif trainingtype == 2:
        print("Train in on multiple sessions, each session with a different target")
        print("*** Learning starts ***")

        agent = sb3.PPO(
            policy="MlpPolicy",
            env=myenv,
            verbose=2,  # 2 = debug
            tensorboard_log="./logdir/",
            # learning_rate=0.01,
            # gamma=0.9
            n_steps=512,
            batch_size=16  # 64 is default
        )

        iterations = 20  # 25 onsim, 20 on real
        timesteps = 512 * iterations
        print(f"Training session {0}; target {target}")
        agent.learn(total_timesteps=timesteps, tb_log_name="PPO_MyEnv_wTarget_0")
        # Note: at the end of learning, the env.reset() will be called by agent.learn
        agent.save(f"./PPO_MyEnv_wTarget_0")
        
        for session in range(1, 100):
            target = randint(0, viewport[0] - 1), randint(0, viewport[1]-1)
            myenv.set_target(target)
            print(f"Training session {session}: target {target}")
            agent.learn(total_timesteps=timesteps, tb_log_name=f"PPO_MyEnv_wTarget_{session}", reset_num_timesteps=True)
            agent.save(f"./PPO_MyEnv_wTarget_{session}")

    else:
        print("trainingtype '{trainigtype}' unknown!")
        
    print("Done")

    
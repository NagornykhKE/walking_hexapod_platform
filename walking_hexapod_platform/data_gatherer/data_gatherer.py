#!/usr/bin/env python3

import math
import os
import random

import numpy as np
import gym

import rospy
from std_msgs.msg import Float64MultiArray
from gazebo_msgs.msg import ModelStates,ModelState
from gazebo_msgs.srv import SetModelState

# Q-Learning environment
class CustomEnv(gym.Env):
    """Custom Environment that follows gym interface"""
    metadata = {'render.modes': ['human']}

    def __init__(self, data_gatherer):
        super(CustomEnv, self).__init__()
        self.data_gatherer = data_gatherer
        # constants:
        # list of directions the robot can move to (waiting + 4 directions with changes of coordinates)
        self.possible_actions = [('use_big_step_dist', [0.2, None, None, None, None]),
                                 ('use_normal_step_dist', [0., None, None, None, None]),
                                 ('use_low_step_dist', [-0.2, None, None, None, None]),
                                 ('use_big_cycle_time', [None, 0.2, None, None, None]),
                                 ('use_normal_cycle_time', [None, 0., None, None, None]),
                                 ('use_low_cycle_time', [None, -0.2, None, None, None]),
                                 ('use_big_leg_y', [None, None, 0.05, None, None]),
                                 ('use_normal_leg_y', [None, None, 0., None, None]),
                                 ('use_low_leg_y', [None, None, -0.05, None, None]),
                                 ('use_big_body_height_down', [None, None, None, 0.04, None]),
                                 ('use_normal_body_height_down', [None, None, None, 0., None]),
                                 ('use_low_body_height_down', [None, None, None, -0.04, None]),
                                 ('use_big_body_height_up', [None, None, None, None, 0.04]),
                                 ('use_normal_body_height_up', [None, None, None, None, 0.]),
                                 ('use_low_body_height_up', [None, None, None, None,-0.04])]
        # robot states: R is roll, P is pitch, n is negative, p is positive
        self.possible_states = [('state_neutral', 0, 0),
                                ('state_Rn', -1, 0),
                                ('state_Rp', 1, 0),
                                ('state_Pn', 0, -1),
                                ('state_Pp', 0, 1),
                                ('state_RnPn', -1, -1),
                                ('state_RpPn', 1, -1),
                                ('state_RnPp', -1, 1),
                                ('state_RpPp', 1, 1)]
    
        # initialize inner variables of the environment
        self.reset()
    
        # required defines:
        # 1. action space: choose one of actions
        self.action_space = gym.spaces.Discrete(len(self.possible_actions))
        # 2. observation space: two angles (roll and pitch);
        # low = 0, big = 1, neutral = 2
        self.observation_space = gym.spaces.Tuple((gym.spaces.Discrete(3),
                                                   gym.spaces.Discrete(3)))

    def reset(self):
        # Reset the state of the environment to an initial state
        return self._calculate_observation()

    def step(self, action):
        # Execute one time step within the environment:
        # change parameters of the gait
        a_name, deltas = self.possible_actions[action]
        new_robot_deltas = self.data_gatherer.current_param_deltas[:]
        for i,x in enumerate(deltas):
            if x is not None:
                new_robot_deltas[i] = x
        self.data_gatherer.current_param_deltas = new_robot_deltas
        self.data_gatherer.current_param_list = [x+y for x,y in zip(self.data_gatherer.current_param_deltas,
                                                                    self.data_gatherer.init_param_list)]
        self.data_gatherer.update_robot_gait()
        # calculate reward
        reward = self._calculate_reward()
        # calculate next observation
        obs = self._calculate_observation()
        # check if processing finished
        done = True
        # return observation and reward
        return obs, reward, done, {}
    
    def render(self, mode='human', close=False):
        pass

    def _calculate_reward(self):
        return self.data_gatherer.current_velocity
    
    def _calculate_observation(self):
        # constants
        roll_threshold = 4. * math.pi / 180.
        pitch_threshold = 4. * math.pi / 180.
        roll = self.data_gatherer.current_roll
        pitch = self.data_gatherer.current_pitch
        relative_roll = 1 if roll < -roll_threshold else (2 if roll > roll_threshold else 3)
        relative_pitch = 1 if pitch < -pitch_threshold else (2 if pitch > pitch_threshold else 3)
        return [relative_roll, relative_pitch]


# ROS gatherer
class DataGatherer(object):
    def __init__(self, qtable_file, robot_name, model_world, 
                 dist_start, dist_end, out_file, 
                 init_param_list, 
                 training_mode, num_orients, num_tests_for_each_orient):
        
        # constants
        self.qtable_file = qtable_file
        self.robot_name = robot_name
        self.model_world = model_world
        self.dist_start = dist_start
        self.dist_end = dist_end
        self.out_file = out_file
        self.param_names = ['step_dist', 'cycle_time', 'leg_y', 'body_height_down', 'body_height_up']
        self.init_param_list = init_param_list
        self.training_mode = training_mode
        # Q-learning constants
        self.alpha = 0.1
        self.gamma = 0.6
        self.epsilon = 0.1
        
        # state variables
        self.current_param_deltas = [0.] * len(init_param_list)
        self.current_param_list = init_param_list[:]
        self.refresh_position = True # True if robot should be replaced in beginning of cb_model_states
        self.time_start = None
        self.time_end = None
        self.curr_orient = None
        # calculate all different used orients
        orient_step = 2 * math.pi / num_orients
        self.remaining_ori_tests = {i*orient_step : num_tests_for_each_orient
                                    for i in range(num_orients)}
        self.current_roll = 0.
        self.current_pitch = 0.
        self.current_velocity = 0.
        self.q_env = CustomEnv(self)
        # Q table
        if os.path.exists(self.qtable_file):
            # load table from file
            self.q_table = np.load(self.qtable_file)
        else:
            self.q_table = np.zeros([len(self.q_env.possible_states),
                                     len(self.q_env.possible_actions)])
        self.current_obs = self.q_env.reset()
        
        # ROS infrastructure:
        # service message
        self.state_msg = ModelState()
        self.state_msg.model_name = self.robot_name
        self.state_msg.pose.position.x = -14
        global x 
        global y 
        x = -14
        y = 0
        self.state_msg.pose.position.y = 0  
        self.state_msg.pose.position.z = 5
        self.state_msg.pose.orientation.x = 0
        self.state_msg.pose.orientation.y = 0
        self.state_msg.pose.orientation.z = 0
        self.state_msg.pose.orientation.w = 1
        # publisher for robot params
        self.params_pub = rospy.Publisher('/set_gait_params', Float64MultiArray, queue_size=10)
        # proxy
        self.set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        rospy.wait_for_service('/gazebo/set_model_state')
        # subscriber
        self.s = rospy.Subscriber('/gazebo/model_states', ModelStates, self.cb_model_states)
    
    def _choose_command(self):
        q_table_ind = 3 * (self.current_obs[0] - 1) + self.current_obs[1] - 1
        if random.uniform(0, 1) < self.epsilon:
            action = self.q_env.action_space.sample()
        else:
            action = np.argmax(self.q_table[q_table_ind])
        next_obs, reward, done, info = self.q_env.step(action)
        old_value = self.q_table[q_table_ind, action]
            
        q_table_new_ind = 3 * (next_obs[0] - 1) + next_obs[1] - 1
        next_max = np.max(self.q_table[q_table_new_ind])
        
        new_value = (1 - self.alpha) * old_value + self.alpha * (reward + self.gamma * next_max)
        if self.training_mode:
            self.q_table[q_table_ind, action] = new_value
        self.current_obs = next_obs

    
    def write_data_to_logfile(self, exec_time):
        # if output file is empty, create it with header row
        if not os.path.exists(self.out_file):
            with open(self.out_file, 'w') as f:
                f.write('world;' + ';'.join(self.param_names) 
                        + ';orient;exec_time;mean_velocity\n')
        # write result to output file
        with open(self.out_file, 'a') as f:
            self.current_velocity = (self.dist_end-self.dist_start)/exec_time
            template = '{};'+'{:.5f};' * (len(self.current_param_list) + 3)+'\n'
            values = [self.model_world] + self.current_param_list + [self.curr_orient,
                                                                     exec_time,
                                                                     self.current_velocity]
            f.write(template.format(*values))
    
    def _restart_robot(self):
        # q-learning
        self._choose_command()
        # move robot to initial position with random orientation; 
        # if all possible positions are already processed, return True
        if len(self.remaining_ori_tests) == 0:
            return True
        # choose any remaining orientation
        self.curr_orient = random.choice(list(self.remaining_ori_tests.keys()))
        # decrease number of tests for this orientation
        self.remaining_ori_tests[self.curr_orient] -= 1
        if self.remaining_ori_tests[self.curr_orient] == 0:
            del self.remaining_ori_tests[self.curr_orient]
        # set robot orientation
        self.state_msg.pose.orientation.z = math.sin(self.curr_orient * 0.5)
        self.state_msg.pose.orientation.w = math.cos(self.curr_orient * 0.5)
        self.set_state(self.state_msg)
        return False

    def _calculate_roll_and_pitch(self, x, y, z, w):
        self.current_pitch = math.asin(-2.0*(x*z - w*y))
        self.current_roll = math.atan2(2.0*(x*y + w*z), w*w + x*x - y*y - z*z)

    def cb_model_states(self, msg):
        # refresh position if necessary
        if self.refresh_position:
            processing_finished = self._restart_robot()
            if processing_finished:
                # save data 
                if self.training_mode:
                    np.save(self.qtable_file, self.q_table)
                rospy.signal_shutdown('gathering finished')
            self.refresh_position = False
            self.time_start = None
            return
        
        # get index of the ant
        if 'walking_hexapod_platform' not in msg.name:
            return
        ant_index = msg.name.index('walking_hexapod_platform')
        pos = msg.pose[ant_index].position
        dist = ((pos.x - x) ** 2 + (pos.y - y) ** 2) ** 0.5
        ori = msg.pose[ant_index].orientation
        self._calculate_roll_and_pitch(ori.x, ori.y, ori.z, ori.w)
        #rospy.logerr('x {:.5f}, y {:.5f}'.format(pos.x, pos.y))
        #rospy.logerr('dist: {:.5f}'.format(dist))
        curr_time = rospy.Time.now()
        # sanity check
        if dist > self.dist_end and self.time_start is None:
            # skip processing after reset
            #rospy.logerr('Model is too far from the origin; error')
            return
            
        if dist > self.dist_start and self.time_start is None:
            self.time_start = curr_time
            #rospy.loginfo('data gathering started')
    
        if dist > self.dist_end:
            #rospy.loginfo('data gathering finished')
            time_end = curr_time
            # generate output
            exec_time = ((time_end - self.time_start).to_sec())
            if exec_time < 1e-4:
                rospy.logerr('model failed')
            else:
                self.write_data_to_logfile(exec_time)
                if self.training_mode:
                    np.save(self.qtable_file, self.q_table)

            self.refresh_position = True

    
    def update_robot_gait(self):
        params_msg = Float64MultiArray()
        params_msg.data = self.current_param_list
        self.params_pub.publish(params_msg)
        

if __name__ == '__main__':
    rospy.init_node('data_gatherer')
    # read parameters
    qtable_file = rospy.get_param('~qtable_file')
    robot_name = rospy.get_param('~robot_name', 'walking_hexapod_platform')
    model_world = rospy.get_param('~model_world')
    dist_start = rospy.get_param('~dist_start', 1)
    dist_end = rospy.get_param('~dist_end', 3)
    out_file = rospy.get_param('~out_file')
    # initial values of gait parameters
    step_dist = rospy.get_param('~step_dist')
    cycle_time = rospy.get_param('~cycle_time')
    leg_y = rospy.get_param('~leg_y')
    body_height_down = rospy.get_param('~body_height_down')
    body_height_up = rospy.get_param('~body_height_up')
    # test parameters
    training_mode = rospy.get_param('~training_mode', True)
    num_orients = rospy.get_param('~num_orients', 16)
    num_tests_for_each_orient = rospy.get_param('~num_tests_for_each_orient', 10)
    # sanity check
    if dist_end <= dist_start:
        rospy.logerr('incorrect start and end distances: {} > {}'.format(dist_start, dist_end))
        exit(-1)
    # create gatherer class
    dg = DataGatherer(qtable_file, robot_name, model_world,
                      dist_start, dist_end, out_file, 
                      [step_dist, cycle_time, leg_y, body_height_down, body_height_up],
                      training_mode, num_orients,num_tests_for_each_orient)
    rospy.spin()
    
    

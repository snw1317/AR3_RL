#Robot Envirnoment
from gym import Env
from gym.spaces import Box
import numpy as np
#import random
import urdfpy as upy


class AR3_Env(Env):
    goal = np.array([0.398,-0.408,0.343,-1.93,0,-2.36]) #x,y,z, rpy
    goalRadius = .005 #3mm 

    def __init__(self):
        #Import AR3 URDF and Initalize at Rest Pose
        self.robot = upy.URDF.load('ar3.urdf') 
        joint1 = 0
        joint2 = 0
        joint3 = 1
        joint4 = 0
        joint5 = 0
        joint6 = 0
        jointPose = np.array([joint1,joint2,joint3,joint4,joint5,joint6])
        self.jointPose = np.deg2rad(jointPose)
        #Actions we can take, will allow for a max of +/- 10 degress 
        low_actions = np.ones(6)*-0.1745329
        high_actions = low_actions*-1
        self.action_space = Box(low=low_actions,high=high_actions,dtype=np.float32)

        low_obs = np.array([-2.96706, -0.698132, 0.0174533, -2.8710666, -1.81776042, -2.5848326, -1.0, -1.0, 0])
        high_obs = np.array([2.96706, 1.570796, 2.5080381, 2.8710666, 1.81776042, 2.5848326, 1.0, 1.0, 1.0])
        self.observation_space = Box(low_obs, high_obs, dtype=np.float32)

        #self.on_goal = 0
        self.episode_length = 200


    def step(self,action):
        #Apply Action to Upadte State
        self.jointPose = np.add(self.jointPose,action)

        #Calculate EE Position 
        fk_current = self.robot.link_fk(cfg={'joint_1': self.jointPose[0], 
                                             'joint_2': self.jointPose[1],
                                             'joint_3': self.jointPose[2],
                                             'joint_4': self.jointPose[3],
                                             'joint_5': self.jointPose[4],
                                             'joint_6': self.jointPose[5]})

        T0_6 = fk_current[self.robot.links[6]]
        #xyzrpy = upy.matrix_to_xyz_rpy(T0_6)

        x = T0_6[0,3]
        y = T0_6[1,3]
        z = T0_6[2,3]
        endEffectorPose = np.array([x,y,z])
        #rx = xyzrpy[3]
        #ry = xyzrpy[4]
        #rz = xyzrpy[5]   
         
        #Calculate ditance to goal
        distance2goal = np.sqrt((self.goal[0]-x)**2 + (self.goal[1]-y)**2 + (self.goal[2]-z)**2)                                
        #Calculate Reward
        reward = -distance2goal

        #Define if Done
        self.episode_length -= 1
        if self.episode_length <= 0:
            done = True
        else:
            done = False

            if distance2goal < self.goalRadius:
                reward += 10
                done = True


    #if distance2goal < self.goalRadius:
    #            reward += 5
     #           self.on_goal += 1
      #          if self.on_goal > 5:
       #             done = True
        #    else:
        #        self.on_goal = 0
        #       done = False

        info = {}
        
        state = np.concatenate((self.jointPose, endEffectorPose),axis=None)

        return state, reward, done, info

    def render(self):
        pass

    def reset(self):
        joint1 = 0
        joint2 = 0
        joint3 = 1
        joint4 = 0
        joint5 = 0
        joint6 = 0
        jointPose = np.array([joint1,joint2,joint3,joint4,joint5,joint6])
        self.jointPose = np.deg2rad(jointPose)

        fk_current = self.robot.link_fk(cfg={'joint_1': self.jointPose[0], 
                                             'joint_2': self.jointPose[1],
                                             'joint_3': self.jointPose[2],
                                             'joint_4': self.jointPose[3],
                                             'joint_5': self.jointPose[4],
                                             'joint_6': self.jointPose[5]})

        T0_6 = fk_current[self.robot.links[6]]

        x = T0_6[0,3]
        y = T0_6[1,3]
        z = T0_6[2,3]
        endEffectorPose = np.array([x,y,z])
        self.episode_length = 200

        

        state = np.concatenate((self.jointPose, endEffectorPose),axis=None)

        return state

if __name__ == '__main__':
    env = AR3_Env()
    episodes = 10
    
    for i in range(episodes+1):
        state = env.reset()
        done = False
        ep_r = 0
        #for j in range(env.episode_length):
        while not done:
            action = env.action_space.sample()
            #print(action)
            n_state, reward, done, info = env.step(action)
            #print(n_state)
            ep_r += reward
            if i < 2:
                print('==============ACTION================')
                print(action)
                print('==============STATE================')
                print(n_state)

        print('Episode:{} ep_r:{} num_steps: {}'.format(i,ep_r, 200-env.episode_length)) 
        #print(action)
           
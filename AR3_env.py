
import numpy as np
import urdfpy as upy


class AR3Env(object):
    goal = np.array([0.388,-0.398,0.236,-1.93,0,-2.36]) #x,y,z, rpy
    goalRadius = .01 #50mm
    state_dim = 19
    action_dim = 6
    action_bound = [-.174, .174] # bound in degree +/-10 deg
    #dt = .1
    
    #Joint Limits for reference 
    joint1_lowLim = -170
    joint1_upLim = 170 
    joint2_lowLim = -129 #AR4 = -42
    joint2_upLim = 0 # AR4 = 90
    joint3_lowLim = 1 #AR4 = -89
    joint3_upLim = 143 #AR4 = 52
    joint4_lowLim = -165
    joint4_upLim = 165
    joint5_lowLim = 105
    joint5_upLim = 105
    joint6_lowLim = -155
    joint6_upLim = 155

    def __init__(self):
        self.robot = upy.URDF.load('ar3.urdf') #Import AR3 URDF model
        #Set robot in Home position in degrees
        joint1 = 0
        joint2 = 0
        joint3 = 1
        joint4 = 0
        joint5 = 0
        joint6 = 0
        jointPose = np.array([joint1,joint2,joint3,joint4,joint5,joint6])
        self.jointPose = np.deg2rad(jointPose)

        self.on_goal = 0

    def step(self,action):
        done = False
        action = np.clip(action, *self.action_bound)
        #print('Action')
        #print(action)

        #add action to current joint positions 
        self.jointPose = np.add(self.jointPose,action)
        #print('JointPose')
        #print(self.jointPose)
        
        #Calc EE position 
        fk_current = self.robot.link_fk(cfg={'joint_1': self.jointPose[0], 
                                             'joint_2': self.jointPose[1],
                                             'joint_3': self.jointPose[2],
                                             'joint_4': self.jointPose[3],
                                             'joint_5': self.jointPose[4],
                                             'joint_6': self.jointPose[5]})

        T0_6 = fk_current[self.robot.links[6]]
        xyzrpy = upy.matrix_to_xyz_rpy(T0_6)
        #print(xyzrpy)
        x = T0_6[0,3]
        y = T0_6[1,3]
        z = T0_6[2,3]
        #endEffectorPose = np.array([x,y,z])
        rx = xyzrpy[3]
        ry = xyzrpy[4]
        rz = xyzrpy[5]
        #print(xyzrpy)
        #print('EE Pose')
        #print(endEffectorPose)

        #Calculate distance to goal

        distance2goal = np.sqrt((self.goal[0]-x)**2 + (self.goal[1]-y)**2 + (self.goal[2]-z)**2)
        rotDist = np.sqrt((self.goal[3]-rx)**2 + (self.goal[4]-ry)**2 + (self.goal[4]-rz)**2)

        #Compute Reward
        reward = -distance2goal - .5*rotDist
        #print(reward)

        if distance2goal < self.goalRadius:
            reward += 5
            self.on_goal += 1
            if self.on_goal > 5:
                done = True
        else:
            self.on_goal = 0

        #state = np.concatenate((jointPose, endEffectorPose,distance2goal),axis=None)
        state = np.concatenate((self.jointPose, xyzrpy,self.goal,[1. if self.on_goal else 0.]),axis=None)
        #print('State:')
        #print(state)

        return state, reward, done
        
    def sample_action(self):
        return (np.random.rand(6)-.5)*.6 

    def reset(self):
        self.on_goal = 0

        joint1 = 0
        joint2 = 0
        joint3 = -25
        joint4 = 0
        joint5 = 0
        joint6 = 0
        self.jointPose = np.array([joint1,joint2,joint3,joint4,joint5,joint6])
        self.jointPose = np.deg2rad(self.jointPose)

        fk_current = self.robot.link_fk(cfg={'joint_1': self.jointPose[0], 
                                             'joint_2': self.jointPose[1],
                                             'joint_3': self.jointPose[2],
                                             'joint_4': self.jointPose[3],
                                             'joint_5': self.jointPose[4],
                                             'joint_6': self.jointPose[5]})

        T0_6 = fk_current[self.robot.links[6]]
        xyzrpy = upy.matrix_to_xyz_rpy(T0_6)
        #x = T0_6[0,3]
        #y = T0_6[1,3]
        #z = T0_6[2,3]
        #rx = xyzrpy[3]
        #ry = xyzrpy[4]
        #rz = xyzrpy[5]
        #endEffectorPose = np.array([x,y,z])

        #distance2goal = np.sqrt((self.goal[0]-x)**2 + (self.goal[1]-y)**2 + (self.goal[2]-z)**2 )

        state = np.concatenate((self.jointPose, xyzrpy, self.goal,[1. if self.on_goal else 0.]),axis=None)
        return state



if __name__ == '__main__':
    env = AR3Env()
    for x in range(10):
        env.step(env.sample_action())

 






































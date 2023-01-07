#AR3 Testing Script
from openAI_AR3_env import AR3_Env
from stable_baselines3 import DDPG, PPO
import numpy as np

env = AR3_Env()

#model = DDPG.load('ddpg_AR3')
model = PPO.load('ppo_AR3')
#env = model.get_env()
episodes = 5

for i in range(episodes+1):
    state = env.reset()
    #print('Reset')
    #print(state)
    done = False
    ep_r = 0
    actions_table = np.zeros((200,6))
    #create numpy array to store actions
    while not done:
        action, _state = model.predict(state)
        state, reward, done, info = env.step(action)
        ep_r += reward
        steps = 200-env.episode_length
        #print(state)
        #store the action in this array  
        actions_table[steps-1] = action


        if i < 1:
            print('==============ACTION================')
            print(action)
            #print('==============STATE================')
            #print(state)
        
    def save_actions(actions_table):
            #use np.savetxt to save the actions table in a text file with the episode number
            np.savetxt('actions_table'+str(i)+'.txt', actions_table, delimiter=',')
    save_actions(actions_table)


    print('Episode: {} ep_r: {} steps: {}'.format(i, ep_r,steps))
    #save actions table after each episode
    #print(state)


  
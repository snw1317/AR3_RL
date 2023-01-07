#AR3 Training Script
from openAI_AR3_env import AR3_Env
from stable_baselines3 import DDPG, PPO

#ON_TRAIN = True

env = AR3_Env()

#DDPG
#model = DDPG("MlpPolicy", env, gamma=0.9, verbose=1)
#model.learn(total_timesteps=150000, log_interval=50)
#model.save("ddpg_AR3")
#env = model.get_env()

model = PPO("MlpPolicy", env, gamma=0.7, verbose=1)
model.learn(total_timesteps=10000000,log_interval=50)
model.save("ppo_AR3")

del model

print('======================Testing================================')
model = PPO.load('ppo_AR3')

episodes = 10

for i in range(episodes+1):
    state = env.reset()
    #print('Reset')
    #print(state)
    done = False
    ep_r = 0
    while not done:
        action, _state = model.predict(state)
        state, reward, done, info = env.step(action)
        ep_r += reward
        #print(state)
    print('Episode: {} ep_r: {}'.format(i, ep_r))
    
    #print(state)
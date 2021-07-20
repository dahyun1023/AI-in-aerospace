import tensorflow.compat.v1 as tf
tf.disable_v2_behavior()
from tensorflow.python.framework import ops
ops.reset_default_graph()
sess = tf.InteractiveSession()

from matplotlib import pyplot as plt
import matplotlib

MAX_EPISODES = 6000
MAX_EP_STEPS = 300
ON_TRAIN     = True

# set env
env   = ArmEnv()
ou_noise = OUActionNoise(mean=np.zeros(1), std_deviation=float(std_dev) * np.ones(1))
s_dim = env.state_dim
a_dim = env.action_dim
a_bound = env.action_bound

# set RL method (continuous)
rl    = DDPG(a_dim, s_dim, a_bound)
steps = []
reward_plt = []
success  = []
distance = []

def wave(x):
        i = x[0];
        j = x[1];
        k = x[2];
        w = np.array([
            [0, -k, j],
            [k, 0, -i],
            [-j, i, 0]])
        return w

def train():
    
    for k in range(1,11):
        # start training
        for i in range(MAX_EPISODES//k):       
            s    = env.reset()
            ep_r = 0.
            ou_noise = OUActionNoise(mean=np.zeros(1), std_deviation=float(std_dev/k) * np.ones(1)) # decreasing noise
            ep_step  = 0
            for j in range(MAX_EP_STEPS):
                a = rl.choose_action(s) + ou_noise()
                s_, r, done = env.step(a)
                rl.store_transition(s, a, r, s_)
                ep_r += r
                if rl.memory_full:
                    # start to learn once has fulfilled the memory
                    rl.learn()

                s = s_

                if done or j == MAX_EP_STEPS-1:
                    print('Ep: %i | %s | ep_r: %.1f | step: %i' % (i, '---' if not done else 'done', ep_r, j+1))
                    reward_plt.append(ep_r)
                    ep_step += 1
                    if done:
                        success.append(1)
                    else:
                        success.append(0)

                    distance.append(np.sqrt(s[6]**2 + s[7]**2))   
                    break
                ep_step += 1

            env.print_figures(ep_step)
        rl.save()


def eval():
    rl.restore()
    s = env.reset()
    while True:
        a = rl.choose_action(s)
        s, r, done = env.step(a)
     
    
if ON_TRAIN:
    train()
    success_rate = []
    for i in range(int(len(success)/100)):
        summ = 0
        for j in range(100):
            summ += success[j + i*100]
        success_rate.append(summ/100)
        
    xaxis = np.arange(len(reward_plt))
        
    plt.figure()
    plt.subplot(511)
    plt.plot(xaxis,np.array(reward_plt))
    plt.title("Rewards per Episode")
        
    plt.subplot(513)
    plt.plot(np.arange(len(success_rate)),np.array(success_rate))
    plt.title("success rate")
    
    plt.subplot(515)
    plt.plot(xaxis,np.array(distance))
    plt.title("Final distance")
    
    plt.show()
    
else:
    eval()
    success_rate = []
    for i in range(int(len(success)/100)):
        summ = 0
        for j in range(100):
            summ += success[j + i*100]
        success_rate.append(summ/100)
        
    xaxis = np.arange(len(reward_plt))
        
    plt.figure()
    plt.subplot(511)
    plt.plot(xaxis,np.array(reward_plt))
    plt.title("Rewards vs Episode")
        
    plt.subplot(513)
    plt.plot(np.arange(len(success_rate)),np.array(success_rate))
    plt.title("success rate")
    
    plt.subplot(515)
    plt.plot(xaxis,np.array(distance))
    plt.title("Final distance")
    
    plt.show()

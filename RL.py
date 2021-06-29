import numpy as np
import pandas as pd
from random import seed
from random import random
import matplotlib.pyplot as plt

P = np.zeros((50,50))
P[0,0] = 0.5
P[0,1] = 0.5
P[49,48] = 0.5
P[49,49] = 0.5
for i in range(48):
    for j in range(50):
        if j == i or j == i+1 or j == i+2:
            P[i+1,j] = 1/3
        else:
            P[i+1,j] = 0
#v = np.ones((1,50))
#v /= v.sum()
v = np.zeros((1,50))
v[0,0] = 1

state = v
stateHist = state
dfStateHist = pd.DataFrame(state)
distr_hist = np.zeros((1,50))

for x in range(30):
  state = np.dot(P,state)
  print(state)
  stateHist = np.append(stateHist,state,axis=0)
  dfDistrHist = pd.DataFrame(stateHist)
  dfDistrHist.plot()
plt.show()

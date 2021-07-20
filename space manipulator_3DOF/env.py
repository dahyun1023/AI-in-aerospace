import numpy as np
from numpy.linalg import inv
from numpy import linalg as LA
import math

class ArmEnv(object):
    viewer = None
    dt     = .1    # refresh rate
    action_bound   = [-1, 1]
    fixed_target   = {'x': 8, 'y': 7, 'l': 0.5} # dictionary
    state_dim      = 8
    action_dim     = 3     # joints angular vel
    
       
    def __init__(self):
        self.body_kin_info = np.zeros(4, dtype=[('l', np.float32),('theta', np.float32),('w', np.float32)]) 
        self.body_kin_info['l']     = [1, 2, 2,2, 2, 1]                  # base to arm, and 2 links length
        self.body_kin_info['theta'] = [0, np.pi/3, np.pi*7/9, np.pi/6]    # one base angle, and two joint angles information
        self.body_kin_info['w']     = 0
        
        self.body_para_info = np.zeros(6, dtype=[('mass', np.float32),('Ix', np.float32),('Iy',np.float32),('Iz',np.float32)])
        self.body_para_info['mass'] = [1000, 20,  20, 20, 20, 10]
        self.body_para_info['Ix']   = [100, 0.1, 0.1, 0.1, 0.1, 0.05]                                     
        self.body_para_info['Iy']   = [100, 0.1, 0.1, 0.1, 0.1, 0.05]
        self.body_para_info['Iz']   = [100,0.05,0.05,0.05,0.05, 0.02]
        
        self.base_info = np.zeros(3, dtype=[('r0',np.float32),('v0',np.float32)])
        self.base_info['r0'] = [ 5., 5., 0]
        self.base_info['v0'] = 0
        
        self.on_goal = 0
        
        self.reward_print = []
        self.r0_x_print   = []
        self.r0_y_print   = []       
        self.pend_x_print = []
        self.pend_y_print = []
        self.target_x_print = []
        self.target_y_print = []
        
        self.we = 0
        
        
    def step(self, action):
        done   = False
        action = np.clip(action, *self.action_bound)        
        (base_a,  a1l, a2l, a3l) = self.body_kin_info['l']        
        (base_th, a1r, a2r, a3r) = self.body_kin_info['theta']    # radian, angle
        
        # coupling kinematics(GJM)
        m    = self.body_para_info['mass']
        Mass = sum(m)        
        (Ix0,Ix1,Ix2, Ix3) = self.body_para_info['Ix']
        (Iy0,Iy1,Iy2, Iy3) = self.body_para_info['Iy']
        (Iz0,Iz1,Iz2, Iz3) = self.body_para_info['Iz']
        
        base_xy= self.base_info['r0']     
        r0     = base_xy        
        a1_xy  = np.array([np.cos(base_th), np.sin(base_th), 0])*base_a + base_xy           # a1 start(p1)
        r1     = np.array([np.cos(base_th + a1r), np.sin(base_th + a1r), 0]) * a1l/2 + a1_xy  
        r1x    = wave(r1)
        a1xy_  = np.array([np.cos(base_th + a1r), np.sin(base_th + a1r), 0]) * a1l   + a1_xy  # a1 end and a2 start (p2)
        theta  = base_th + a1r + a2r
        r2     = np.array([np.cos(theta), np.sin(theta), 0]) * a2l/2 + a1xy_
        r2x    = wave(r2)
        p3     = np.array([np.cos(theta), np.sin(theta), 0]) * a2l   + a1xy_  # a2 end (x2, y2)
        theta  = base_th + a1r + a2r + a3r
        r3     = np.array([np.cos(theta), np.sin(theta), 0]) * a3l/2 + p3
        r3x    = wave(r3)
        p_end  = np.array([np.cos(theta), np.sin(theta), 0]) * a3l  + p3
        
        E3     = np.identity(3)
        rg     = (base_xy*m[0] + r1*m[1] + r2*m[2] + r3*m[3])/Mass
        rgx    = wave(rg)
        r0g    = rg - base_xy
        r0gx   = wave(r0g)
        z      = np.array([0,0,1])
        A      = np.array([np.cross(z,r1 - a1_xy)])
        B      = np.array([[0,0,0]])
        C      = B
        JP1    = np.concatenate((A.T, B.T, C.T), axis = 1) # 가로
        A      = np.array([np.cross(z,r2 - a1_xy)])
        B      = np.array([np.cross(z,r2 - a1xy_)])
        JP2    = np.concatenate((A.T, B.T, C.T), axis = 1) # 가로
        A      = np.array([np.cross(z,r3 - a1_xy)])
        B      = np.array([np.cross(z,r3 - a1xy_)])
        C      = np.array([np.cross(z,r3 -  p3  )])
        JP3    = np.concatenate((A.T, B.T, C.T), axis = 1) # 가로        
        JPphi  = m[1]*JP1 + m[2]*(JP1 + JP2) + m[3]*(JP1 + JP2+ JP3)

        JH1  = np.array([
            [0,0,0],
            [0,0,0],
            [1,0,0]])
        JH2  = np.array([
            [0,0,0],
            [0,0,0],
            [1,1,0]])
        JH3  = np.array([
            [0,0,0],
            [0,0,0],
            [1,1,1]])
        Pphi = np.array([
            [Ix1,0,0],
            [0,Iy1,0],
            [0,0,Iz1]]).dot(JH1) + np.array([
            [Ix2,0,0],
            [0,Iy2,0],
            [0,0,Iz2]]).dot(JH2) + np.array([
            [Ix3,0,0],
            [0,Iy3,0],
            [0,0,Iz3]]).dot(JH3) + m[1]*r1x.dot(JP1) + m[2]*r2x.dot(JP2) + m[3]*r3x.dot(JP3)
        r01  = r1 - r0
        r02  = r2 - r0
        r03  = r3 - r0
        r01x = wave(r01)
        r02x = wave(r02)
        r03x = wave(r03)
        Pw   = np.array([
            [Ix0+Ix1+Ix2+Ix3,0,0],
            [0,Iy0+Iy1+Iy2+Iy3,0],
            [0,0,Iz0+Iz1+Iz2+Iz3]]) - m[1]*r1x.dot(r01x) - m[2]*r2x.dot(r02x) - m[3]*r3x.dot(r03x)

        # eq 12
        A     = np.concatenate((Mass*E3,Mass*rgx),axis = 0) # 세로 합치기
        B     = np.concatenate((-Mass*r0gx,Pw),axis = 0)    # 세로 합치기
        C     = np.concatenate((A,B),axis = 1)              # 가로 합치기  
        D     = np.concatenate((JPphi,Pphi),axis = 0)       # 세로 합치기
        E     = -np.linalg.inv(C).dot(D)
        v0_w0 = E.dot(action)

        v0    = np.array([v0_w0[0], v0_w0[1], v0_w0[2]]) 
        w0    = v0_w0[5]
        
        # eq 5
        A     = np.concatenate((E3,np.zeros((3,3))),axis = 0) # 세로 합치기
        p0e   = p_end - base_xy
        p0ex  = wave(p0e)
        B     = np.concatenate((-p0ex,E3),axis = 0)         # 세로 합치기
        J0    = np.concatenate((A,B),axis = 1)              # 가로 합치기

        # eq 7
        z_x   = wave(z)
        A     = z_x.dot(np.array([
                    p_end[0]-a1_xy[0],
                    p_end[1]-a1_xy[1],
                    p_end[2]-a1_xy[2]]))
        B     = z_x.dot(np.array([
                    p_end[0]-a1xy_[0],
                    p_end[1]-a1xy_[1],
                    p_end[2]-a1xy_[2]]))
        C     = z_x.dot(np.array([
                    p_end[0]-p3[0],
                    p_end[1]-p3[1],
                    p_end[2]-p3[2]]))
        A     = np.array([
                [A[0],B[0],C[0]],
                [A[1],B[1],C[1]],
                [A[2],B[2],C[2]]])   # 가로 합치기
        B     = np.array([
                [0,0,0],
                [0,0,0],
                [1,1,1]])   
        Jphi  = np.concatenate((A,B),axis = 0)               # 세로 합치기
        F     = Jphi + J0.dot(E)
        ve_we = F*np.transpose(action)
        
        # next state
        W = np.array([w0, action[0]/10, action[1]/10, action[2]/10])       # 가로 합치기
        
        #print(self.body_kin_info['theta'])
        self.body_kin_info['theta'] += W*self.dt
        a = self.body_kin_info['theta'][0]
        self.body_kin_info['theta'] = np.clip(self.body_kin_info['theta'], -1.5708, 1.5708)    
        self.body_kin_info['theta'] = [a, self.body_kin_info['theta'][1],self.body_kin_info['theta'][2],self.body_kin_info['theta'][3]]
        self.body_kin_info['w'] = W
        self.base_info['r0']   += v0 * self.dt
        self.base_info['v0']    = v0
        ve = np.array([ve_we[0],ve_we[1],ve_we[2]])
        we_prev = self.we
        print(f'we_prv:{we_prev}')
        self.we = ve_we[-1]
        print(f've_we:{ve_we[3:]}')
        we_diff = abs(we_prev - self.we)
        print(f'we_diff:{we_diff}')
        # normalize features
        dist = np.array([(self.fixed_target['x'] - p_end[0])/10, (self.fixed_target['y'] - p_end[1])/10])
               
        # done and reward
        r = math.sqrt(dist[0]**2 + dist[1]**2)  
        #r_v = math.sqrt(ve[0][0]**2 + ve[0][1]**2)
        r = -math.log10(10*r + 10**(-8)) #- we_diff
        if self.fixed_target['x'] - self.fixed_target['l']/2     < p_end[0] < self.fixed_target['x'] + self.fixed_target['l']/2:
            if self.fixed_target['y'] - self.fixed_target['l']/2 < p_end[1] < self.fixed_target['y'] + self.fixed_target['l']/2:
                r += 1
                self.on_goal += 1
                if self.on_goal > 20:
                    done = True
        else:
            self.on_goal = 0
        
        # state
        s = np.append(a1xy_,p_end)
        s = np.append(s,dist*10)
        
        # visualization
        self.reward_print = np.append(self.reward_print, r)
        self.r0_x_print = np.append(self.r0_x_print, r0[0])
        self.r0_y_print = np.append(self.r0_y_print, r0[1])
        
        self.pend_x_print = np.append(self.pend_x_print, p_end[0])
        self.pend_y_print = np.append(self.pend_y_print, p_end[1])
        
        self.target_x_print = np.append(self.target_x_print, self.fixed_target['x'])
        self.target_y_print = np.append(self.target_y_print, self.fixed_target['y'])
        
        return s, r, done
    
    def reset(self):       
        self.we = 0      
        self.body_kin_info['w']     = 0
        self.base_info['v0']        = 0
        self.body_kin_info['theta'] = [0, (np.random.rand()-0.5), (np.random.rand()-0.5),(np.random.rand()-0.5)] # less random
        self.on_goal              = 0
        (base_a,  a1l, a2l, a3l)  = self.body_kin_info['l']
        (base_th, a1r, a2r, a3r)  = self.body_kin_info['theta']
        
        self.base_info['r0'] = [ 5., 5., 0]
        base_xy= self.base_info['r0']
        r0     = base_xy
        
        a1_xy  = np.array([np.cos(base_th), np.sin(base_th), 0])*base_a + base_xy           # a1 start(p1)
        r1     = np.array([np.cos(base_th + a1r), np.sin(base_th + a1r), 0]) * a1l/2 + a1_xy  
        r1x    = wave(r1)
        a1xy_  = np.array([np.cos(base_th + a1r), np.sin(base_th + a1r), 0]) * a1l   + a1_xy  # a1 end and a2 start (p2)
        theta  = base_th + a1r + a2r
        r2     = np.array([np.cos(theta), np.sin(theta), 0]) * a2l/2 + a1xy_
        r2x    = wave(r2)
        p3     = np.array([np.cos(theta), np.sin(theta), 0]) * a2l   + a1xy_  # a2 end (x2, y2)
        theta  = base_th + a1r + a2r + a3r
        r3     = np.array([np.cos(theta), np.sin(theta), 0]) * a3l/2 + p3
        r3x    = wave(r3)
        p_end  = np.array([np.cos(theta), np.sin(theta), 0]) * a3l  + p3
                       
        # normalize features
        dist = np.array([(self.fixed_target['x'] - p_end[0])/10, (self.fixed_target['y'] - p_end[1])/10 ])
                
        # state
        s = np.append(a1xy_,p_end)
        s = np.append(s,dist*10)
        
        #visualization
        self.reward_print   = []
        self.r0_x_print     = []
        self.r0_y_print     = []       
        self.pend_x_print   = []
        self.pend_y_print   = []
        self.target_x_print = []
        self.target_y_print = []
        return s
    
    def print_figures(self,MAX_EP_STEPS):
        #plt.figure()
        #plt.plot(np.arange(self.dt,MAX_EP_STEPS*self.dt,self.dt),self.reward_print)
        
        plt.figure()
        plt.plot(self.r0_x_print,self.r0_y_print)
        plt.plot(self.r0_x_print[0],self.r0_y_print[0], marker='*')
        plt.plot(self.r0_x_print[-1],self.r0_y_print[-1], marker='o')
        
        plt.plot(self.pend_x_print,self.pend_y_print, color = 'red')
        plt.plot(self.pend_x_print[0],self.pend_y_print[0], marker='*')
        plt.plot(self.pend_x_print[-1],self.pend_y_print[-1], marker='o')
        
        plt.plot(self.fixed_target['x'],self.fixed_target['y'], marker="x")
        plt.legend(["r0","p_end"])
        plt.show()
        
    def render(self):
        pass
    
    def sample_action(self):
        return np.random.rand(3)-0.5    # three radians

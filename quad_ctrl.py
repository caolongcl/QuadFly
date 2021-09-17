import numpy as np
import matplotlib.pyplot as plt

class Quadfly():
    def __init__(self):
        self.l1 = 0.209
        self.l2 = 0.180

        self.angleh_s = np.math.pi / 4.
        self.anglek_s = - 0.5 * np.math.pi

        # 前肘后膝
        self.leg_type = [1,1,1,1,1,-1,1,-1]
        self.angle_init = [1,1,1,1,-1,-1,-1,-1]

        # 前肘后肘
        self.leg_type = [1,1,1,1,1,1,1,1]
        self.angle_init = [1,1,1,1,1,1,1,1]
        
        #L = 0.70 *(self.l1+self.l2)
        #self.angleh_s = np.arccos((L**2 + self.l1**2 - self.l2**2)/(2*L*self.l1))
        #self.anglek_s = -np.abs(np.arccos((L**2 - self.l1**2 - self.l2**2)/(2*self.l1*self.l2)))
        #print('angleh_s', self.angleh_s, 'anglek_s', self.anglek_s)

        self.mu = 1
        self.alpha = 100
        self.a=100
        self.omega_sw=5*np.math.pi
        self.hk=1
        # pace:beta, (phi_LF, phi_RLF, phi_RH, phi_LH), v, h 
        self.walk_config={'name':'walk', 'beta':0.75, 'phi':[0,0.5,0.25,0.75], 'v':0.1, 'h':0.02}
        self.trot_config={'name':'trot', 'beta':0.5, 'phi':[0,0.5,0,0.5], 'v':0.5, 'h':0.04}
        self.pace_config={'name':'pace', 'beta':0.5, 'phi':[0,0.5,0.5,0], 'v':0.7,  'h':0.02}
        self.gallop_config={'name':'gallop', 'beta':0.5, 'phi':[0,0.1,0.6,0.5], 'v':1.0, 'h':0.04}
        self.stand_config={'name':'stand', 'beta':0.5, 'phi':[0,0.5,0,0.5], 'v':0.0, 'h':0.04}
        #self.bound_config={'name':'bound', 'beta':0.5, 'phi':[0,0.,0.5,0.5], 'v':0.3,  'h':0.02}
        #self.pronk_config={'name':'pronk', 'beta':0.75, 'phi':[0,0,0,0], 'v':0.3, 'h':0.02}

        configs = [self.walk_config, self.trot_config, self.pace_config, self.gallop_config, self.stand_config]
        for config in configs:
            omega_st = self.fomega_st(config['beta'])
            config['T'] = np.math.pi/self.omega_sw + np.math.pi/omega_st
            config['Ah'] = self.Ah(config['beta'], config['v'], config['T'])
            config['Ak'] = self.Ak(config['h'])
            self.config_info(config)

        #print(0.5*0.5*1.4*0.4 / np.sqrt(self.l1**2+self.l2**2-2*self.l1*self.l2*np.cos(self.anglek_s+self.angleh_s)))

        # sim
        self.step=0.001
        period=4
        self.steps = np.arange(0, period, self.step)
        #q = np.array([0.1,0,0,0,0,0,0,0])
        #q = np.array([0.001,0.001,0,0,0,0,0,0])
        q = np.array([0.001,0.001,0,0,0,0,0,0])
        self.Q = np.reshape(q, (8,1))
    
    def config_info(self, config):
        print('----- ', config['name'], ' -----')
        print('beta:', config['beta'], 'v:', config['v'],'m/s', 'h:', config['h'], 'm', 'T:', config['T'], 's',
        'Ah:', np.math.degrees(config['Ah']), 'Ak', np.math.degrees(config['Ak']))

    def Ah(self, beta, v, T):
        return np.arcsin(0.5*beta*v*T / np.sqrt(self.l1**2+self.l2**2-2*self.l1*self.l2*np.cos(self.anglek_s+self.angleh_s)))
    
    def Ak(self, h):
        return np.arccos(np.cos(self.angleh_s+self.anglek_s)-h/self.l2)+(self.angleh_s+self.anglek_s)
    
    def fomega_st(self, beta):
        return ((1-beta)/beta)*self.omega_sw

    def fomega(self, beta, L):
        omega_st = self.fomega_st(beta)
        L1 = L[1,0]
        return omega_st/(np.exp(-self.a*L1)+1) + self.omega_sw/(np.exp(self.a*L1)+1)
    
    def angle_h(self, r, Ah):
        return -r * Ah
    
    def angle_k(self, r, Ak):
        if r <= 0:
            return Ak*r
        else:
            return 0.
    
    def angle(self, Q, config):
        ang = []
        for i in range(len(Q)):
            if i % 2 == 0:
                ang.append(self.angle_h(Q[i,0], config['Ah']))
            else:
                ang.append(self.angle_k(Q[i,0], config['Ak']))
        for i in range(8):
            ang[i] = ang[i] * self.leg_type[i]
        return ang
    
    def Mi(self, L, omega):
        p = self.alpha*(self.mu-np.dot(L.T, L)[0,0])
        return np.array([[p, -omega],[omega, p]])

    def Ri(self, phi, i, j):
        theta = 2*np.math.pi*(phi[i]-phi[j])
        return np.array([[np.cos(theta), -np.sin(theta)],[np.sin(theta), np.cos(theta)]])
    
    def R(self, phi):
        Rall=[]
        for i in range(4):
            Row=[]
            for j in range(4):
                Row.append(self.Ri(phi, i, j))
            
            Row = np.c_[Row[0], Row[1], Row[2], Row[3]]
            Rall.append(Row)
        return np.r_[Rall[0], Rall[1], Rall[2], Rall[3]]

    def hopf(self, Q, beta, phi):
        M = []
        for i in range(4):
            omega = self.fomega(beta, Q[i*2:(i+1)*2,:])
            Mitmp = np.dot(self.Mi(Q[i*2:(i+1)*2,:], omega), Q[i*2:(i+1)*2,:])
            M.append(Mitmp)
        F = np.r_[M[0], M[1], M[2], M[3]]
        return F + np.dot(self.R(phi), Q)

    def run(self, config):    
        ang=[]
        for i in range(len(self.steps)):
            output = self.hopf(self.Q, config['beta'], config['phi'])
            self.Q = output*self.step + self.Q
            
            ang.append(self.angle(self.Q, config))
        ret = np.array(ang)
        return ret
    
    def step_ctrl(self, step, config):
        output = self.hopf(self.Q, config['beta'], config['phi'])
        self.Q = output*step + self.Q
        
        angles = self.angle(self.Q, config)
        for i in range(4):
            angles[i*2] = self.angleh_s * self.angle_init[i*2] +angles[i*2]
            angles[i*2 + 1] = self.anglek_s*self.angle_init[i*2+1] + angles[i*2 + 1]
        return angles
    
if __name__ == "__main__":
    quad = Quadfly()
    ret = quad.run(quad.walk_config)

    step=0.001
    period=4
    steps = np.arange(0, period, step)

    titles = ['LF', 'RF', 'RH', 'LH']
    fig, ax = plt.subplots(4,1, figsize=(16, 16))
    for i in range(4):
        ax[i].plot(steps, ret[:,i*2], 'r', label='ang_h')
        ax[i].plot(steps, ret[:,i*2+1], 'b', label='ang_k')
        ax[i].legend()
        ax[i].set_title(titles[i])
    plt.plot()
    plt.show()
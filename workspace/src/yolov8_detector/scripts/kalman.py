import numpy as np

class KalmanFilter():
    def __init__(self, dt, u_x, u_y, std_acc, x_std_meas, y_std_meas):
        """ 
        :dt: 时间步长 
        :u_x: x方向的加速度控制 
        :u_y: y方向的加速度控制 
        :std_acc: 过程噪声的标准差 
        :x_std_meas: x方向测量的标准差 
        :y_std_meas: y方向测量的标准差 
        """

        self.dt = dt
        self.u = np.matrix([[u_x],[u_y]])
        #initial state guess
        self.x = np.matrix([[0], [0], [0], [0]])
        
        self.A = np.matrix([[1, 0, self.dt, 0],
                            [0, 1, 0, self.dt],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]])
        
        self.B = np.matrix([[(self.dt**2)/2, 0],
                            [0, (self.dt**2)/2],
                            [self.dt,0],
                            [0,self.dt]])
        
        self.H = np.matrix([[1, 0, 0, 0],
                            [0, 1, 0, 0]])

        self.Q = np.matrix([[(self.dt**4)/4, 0, (self.dt**3)/2, 0],
                            [0, (self.dt**4)/4, 0, (self.dt**3)/2],
                            [(self.dt**3)/2, 0, self.dt**2, 0],
                            [0, (self.dt**3)/2, 0, self.dt**2]]) * std_acc**2
        
        self.R = np.matrix([[x_std_meas**2,0],
                           [0, y_std_meas**2]])
        
        #initial cov matrix can be initialized as an identity matrix
        self.P = np.eye(self.A.shape[1])

    def predict(self):
        # 预测下一时刻的状态 
        self.x = np.dot(self.A, self.x) + np.dot(self.B, self.u)
        # 更新状态协方差 
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q
        return self.x[0:2]
    
    def filter(self, z):
        # 计算卡尔曼增益 
        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        # 更新状态向量 
        self.x = np.round(self.x + np.dot(K, (z - np.dot(self.H, self.x))))
        I = np.eye(self.H.shape[1])
        # 计算更新后的状态协方差 
        self.P = (I - (K * self.H)) * self.P
        return self.x[0:2]
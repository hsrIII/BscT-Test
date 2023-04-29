import numpy as np
import Operators

class Testdata:
    def __init__(self, print_case = False, signal_length = 100, sampling_rate = 0.01, _qOp = Operators.Operators()):
        self.print_case = print_case
        self.signal_length = signal_length   # in seconds
        self.sampling_rate = sampling_rate
        self._qOp = _qOp
        
        

    def orientation_testcases(self, case = "A"):
        if case == "A":
            attitudes = np.array([[1,0,0,0],[1,0,0,0]])
            t_ = np.array([0,1])
            
        if case == "B":
            attitudes = np.array([[1,0,0,0],[0,1,0,0]])
            t_ = np.array([0,1])
            
        if case == "C":
            attitudes = np.array([[1,0,0,0],[0,-1,0,0]])
            t_ = np.array([0,1])
        
        if case == "D":
            attitudes = np.array([[1,0,0,0],[1/np.sqrt(2),0,1/np.sqrt(2),0]])
            t_ = np.array([0,1])
        
        if case == "E":
            rec_length = self.signal_length
            step = self.sampling_rate
            t_= np.arange(rec_length, step=step)
            freq_sin = .2
            
            phi = freq_sin*t_
            theta = freq_sin/11*t_
            omega = t_*0.123
            attitudes = self._qOp.spherecoord_in_q(omega, theta, phi)
            
        if case == "F":
            rec_length = self.signal_length
            step = self.sampling_rate
            t_= np.arange(rec_length, step=step)
            freq_sin = .2

            phi = 0*t_
            theta = 0*t_
            omega = t_*-0.0123+np.pi
            omega[0]=0
            attitudes = _qOp.spherecoord_in_q(omega, theta, phi)
        
        if case == "G":
            step = self.sampling_rate
            t_ = np.arange(self.sampling_rate*5, step = self.sampling_rate)
            omega = np.array([0,0.25, 0.5, 0.75, 1])*np.pi/2
            theta = omega
            phi = omega
            attitudes = _qOp.spherecoord_in_q(omega, theta, phi)

        if self.print_case == True:
            print(f"------------------------------------------------------------------ Att. Testcase {case}:\nattitudes:\n {attitudes.round(2)}, \nt_array:\n {t_}")
        return attitudes, t_


    def wtestcases(self, case = "A"):
        if case == "A":
            w_ = np.array([[0,0,0],[0,0,0]])
            t_ = np.array([0,1])
            
        if case == "B":
            w_ = np.array([[0,0,0],[np.pi,0,0]])
            t_ = np.array([0,1])
            
        if case == "C":
            w_ = np.array([[0,0,0],[np.pi,0,0],[np.pi,0,0],[np.pi,0,0]])
            t_ = np.array([0,1,2,3])
            
        if case == "D":
            w_ = np.array([[np.pi,0,0],[np.pi,0,0],[np.pi,0,0],[np.pi,0,0]])
            t_ = np.array([0,1,2,3])
            
        if case == "E":
            w_ = np.array([[np.pi/2,0,0],[0,np.pi/2,0],[0,-np.pi/2,0],[-np.pi/2,0,0]])
            t_ = np.array([0,1,2,3])

        if case == "F": #oneaxial data gyro
            w_ = np.array([[0,0,0],[0,0,0],[0,0,100],[0,0,100],[0,0,100]])
            t_ = np.arange(self.sampling_rate*5, step = self.sampling_rate)

        if case == "G":
            w_ = np.array([np.ones(100),np.zeros(100), np.zeros(100)]).T
            t_ = np.arange(self.sampling_rate*100, step=self.sampling_rate)

        if self.print_case == True: 
            print(f"++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 01 Q Testcase {case}:\nAngular velocities:\n {w_.round(2)}, \nt_array:\n {t_}")
        return w_, t_
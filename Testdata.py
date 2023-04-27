import numpy as np
import Operators as _qOp

def orientation_testcases(case = "A"):
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
        rec_length = 100
        step = 0.01
        t_= np.arange(rec_length, step=step)
        freq_sin = .2
        
        phi = freq_sin*t_
        theta = freq_sin/11*t_
        omega = t_*0.123
        attitudes = _qOp.spherecoord_in_q(omega, theta, phi)
        
    if case == "F":
        rec_length = 1000
        step = 0.01
        t_= np.arange(rec_length, step=step)
        freq_sin = .2

        phi = 0*t_
        theta = 0*t_
        omega = t_*-0.0123+np.pi
        omega[0]=0
        attitudes = _qOp.spherecoord_in_q(omega, theta, phi)
    
    if case == "G":
        step = 0.01
        t_ = np.array([0.0, 0.01, 0.02, 0.03, 0.04])
        omega = np.array([0,0.25, 0.5, 0.75, 1])*np.pi/2
        theta = omega
        phi = omega
        attitudes = _qOp.spherecoord_in_q(omega, theta, phi)

    print(f"++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 01 Q Testcase {case}:\nattitudes:\n {attitudes.round(2)}, \nt_array:\n {t_}")
    return attitudes, t_


def wtestcases(case = "A"):
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
        t_ = np.arange(0.05, step = 0.01)

    if case == "G":
        w_ = np.array([np.ones(100),np.zeros(100), np.zeros(100)]).T
        t_ = np.arange(1,step=0.01)

    print(f"++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 01 Q Testcase {case}:\nAngular velocities:\n {w_.round(2)}, \nt_array:\n {t_}")
    return w_, t_
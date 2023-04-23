import numpy as np
import matplotlib.pyplot as plt
from decimal import Decimal

def forwardoperator(attitudes, t_list):
    gyr_data = np.zeros((attitudes.shape[0],attitudes.shape[1]-1))
    
    for i in range(1, len(gyr_data)):
        dt = t_list[i] - t_list[i - 1]
        gyr_data[i] = q_to_angular_velocity(attitudes[i - 1], attitudes[i], dt)

    return gyr_data, t_list


def q_to_angular_velocity(q1, q2, dt): #https://mariogc.com/post/angular-velocity-quaternions/; q1 = qt, q2 = qt+1
    omega11 = q1[0] * q2[0] + q1[1] * q2[1] + q1[2] * q2[2] + q1[3] * q2[3]
    omega21 = q1[0] * q2[1] - q1[1] * q2[0] - q1[2] * q2[3] + q1[3] * q2[2] 
    omega31 = q1[0] * q2[2] + q1[1] * q2[3] - q1[2] * q2[0] - q1[3] * q2[1] 
    omega41 = q1[0] * q2[3] - q1[1] * q2[2] + q1[2] * q2[1] - q1[3] * q2[0]
    
    w_norm = np.arccos(omega11)*2/dt
    
    if np.sin(w_norm*dt/2) == 0:                    #L'Hospital
        wx = 1/(np.cos(w_norm*dt/2))*2/dt *omega21
        wy = 1/(np.cos(w_norm*dt/2))*2/dt *omega31
        wz = 1/(np.cos(w_norm*dt/2))*2/dt *omega41
    
    else:
        wx = omega21 * w_norm * 1/np.sin(w_norm*dt/2)
        wy = omega31 * w_norm * 1/np.sin(w_norm*dt/2)
        wz = omega41 * w_norm * 1/np.sin(w_norm*dt/2)
    
    angular_velocity = np.array([wx,wy,wz])
    return angular_velocity


def simulate_sensor_events(gyr_data, t_list):
    attitudes = np.zeros((gyr_data.shape[0],gyr_data.shape[1]+1))
    timestamps = np.array([])

    for index in range(gyr_data.shape[0]):
        t = t_list[index]
        if index == 0:
            t_prev = 0
            q = np.array([1,0,0,0])             #Annahme: q(t=0)=[1,0,0,0]
            
        else:
            t_prev = timestamps[-1]

        wt = gyr_data[index,:]
        dt = t-t_prev
        
        q = integrator(wt, q, dt, t)[0]
        timestamps = np.append(timestamps, t)  

        attitudes[index,:] = q

    return attitudes, timestamps


def integrator(wt, q, dt, t=None):
    step = dt
    wx = wt[0]
    wy = wt[1]
    wz = wt[2]

    w_norm = np.sqrt(np.dot(wt,wt))
    Omega = np.array([[0,-wx,-wy,-wz],[wx,0,wz,-wy],[wy,-wz,0,wx],[wz,wy,-wx,0]])
    Eins = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])

    if w_norm == 0:
        q_res = q

    else:
        q_res = (np.cos(w_norm*step/2)*Eins+(1/w_norm)*np.sin(w_norm*step/2)*Omega)@q 
        norm_q_res = np.sqrt(np.dot(q_res,q_res))
        q_res = q_res/norm_q_res

    return q_res, t

def spherecoord_in_q(omega, theta, phi): # Bahn auf Kugeloberfläche, umgerechnet zu Attitude (Quaternions)
    ux = np.sin(theta) * np.cos(phi)   
    uy = np.sin(theta) * np.sin(phi)
    uz = np.cos(theta)

    qw = np.cos(omega)
    qi = np.sin(omega) * ux
    qj = np.sin(omega) * uy
    qk = np.sin(omega) * uz
    attitudes = np.array([qw, qi, qj, qk]).T

    return attitudes

def attitudes_to_acc_data(attitudes):
    acc_data = np.zeros((attitudes.shape[0],3))
    gravity_vector = np.array([0,0,-9.81])

    for index in range(attitudes.shape[0]):
        q = attitudes[index,:]
        rot_matrix = quaternion_to_rotation_matrix(q)
        acc_data[index,:] = np.linalg.inv(rot_matrix)@gravity_vector

    return acc_data

def quaternion_to_rotation_matrix(q): #https://automaticaddison.com/how-to-convert-a-quaternion-to-a-rotation-matrix/
    """
    Covert a quaternion into a full three-dimensional rotation matrix.

    Input
    :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3)

    Output
    :return: A 3x3 element matrix representing the full 3D rotation matrix.
             This rotation matrix converts a point in the local reference
             frame to a point in the global reference frame.
    """
    qw = q[0]
    qi = q[1]
    qj = q[2]
    qk = q[3]

    # First row of the rotation matrix
    r00 = 2 * (qw * qw + qi * qi) - 1
    r01 = 2 * (qi * qj - qw * qk)
    r02 = 2 * (qi * qk + qw * qj)

    # Second row of the rotation matrix
    r10 = 2 * (qi * qj + qw * qk)
    r11 = 2 * (qw * qw + qj * qj) - 1
    r12 = 2 * (qj * qk - qw * qi)

    # Third row of the rotation matrix
    r20 = 2 * (qi * qk - qw * qj)
    r21 = 2 * (qj * qk + qw * qi)
    r22 = 2 * (qw * qw + qk * qk) - 1

    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])

    return rot_matrix

def plot_res(attitudes, w_, q_res, t_ = None, error = []):
    fig, axs = plt.subplots(2,2, figsize=(13, 7))
    plt.rcParams['figure.constrained_layout.use'] = True
    plt.suptitle("Attitude, Angular Velocity and Error vectors")
    axs[0, 0].plot(t_, attitudes, label=['w', 'i', 'j', 'k'], linestyle = "-")
    axs[0, 0].set_title("reference attitude (as quaternions)")
    axs[0, 0].legend()
    axs[0, 0].grid(True)
    axs[0, 0].set_xlabel(r"time $t$")
    axs[0, 0].set_ylabel(r"$\mathbf{q_{ref}}$")

    axs[0, 1].plot(t_, q_res, label=["w res", "i res", "j res", "k res"], linestyle = "-")
    axs[0, 1].legend()
    axs[0, 1].set_title("calculated attitude from angular velocity $\mathbf{w}$ (results)")
    axs[0, 1].grid(True)
    axs[0, 1].set_xlabel(r"time $t$")
    axs[0, 1].set_ylabel(r"$\mathbf{q_{res}}$")

    axs[1, 0].plot(t_, w_, label=["wx", "wy", "wz"])
    axs[1, 0].set_title("angular velocity vectors")
    axs[1, 0].legend()
    axs[1, 0].grid(True)
    axs[1, 0].set_xlabel(r"time $t$")
    axs[1, 0].set_ylabel(r"$\mathbf{w}$")

    if error == []:
        error = attitudes-q_res

    axs[1, 1].plot(t_, error, label=["w", "i", "j", "k"])
    axs[1, 1].set_title(f"Integration Error")
    axs[1, 1].legend()
    axs[1, 1].grid(True)
    axs[1, 1].set_xlabel(r"time $t$")
    axs[1, 1].set_ylabel(r"$\mathbf{q_{ref}}-\mathbf{q_{res}}$")


    fig2, axs2 = plt.subplots(attitudes.shape[1],3, figsize=(13, 7))
    plt.rcParams['figure.constrained_layout.use'] = True
    plt.suptitle("$q_w$, $q_i$, $q_j$, $q_k$ individually")
    for row in range(attitudes.shape[1]):
        axs2[row, 0].plot(t_, attitudes[:, row], label=["w", "i", "j", "k"][row])
        axs2[row, 0].plot(t_, q_res[:,row], label=["w res", "i res", "j res", "k res"][row], linestyle = "--")
        axs2[row, 0].set_title(f"$q_{['w', 'i', 'j', 'k'][row]}$:  reference attitude")
        axs2[row, 0].grid(True)
        axs2[row, 0].legend()
        axs2[row, 0].set_xlabel(r"time $t$")
        axs2[row, 0].set_ylabel(f"$q_{['w', 'i', 'j', 'k'][row]}$")

        error_wijk = attitudes[:, row]-q_res[:,row]
        axs2[row, 1].plot(t_, error_wijk, label=["w", "i", "j", "k"][row])
        axs2[row, 1].set_title(f"max(Err)={'%.1e' % Decimal(max(abs(error_wijk)))}, E[0]={'%.1e' % Decimal(error_wijk[0])}, max(E)-min(E)={'%.1e' % Decimal(max(error_wijk)-min(error_wijk))}")
        axs2[row, 1].grid(True)
        axs2[row, 1].legend()
        axs2[row, 1].set_xlabel(r"time $t$")
        axs2[row, 1].set_ylabel(f"$q_{{{str(['w', 'i', 'j', 'k'][row]+',ref')}}}-q_{{{str(['w', 'i', 'j', 'k'][row]+',res')}}}$")

        axs2[row, 2].plot(t_, attitudes[:, row], label=["w", "i", "j", "k"][row])
        axs2[row, 2].plot(t_, max(attitudes[:,row])/max((error_wijk))*error_wijk, label = "error")
        axs2[row, 2].set_title(f"Error scaled to same amplitude as ref. att.")
        axs2[row, 2].grid(True)
        axs2[row, 2].legend()
        axs2[row, 2].set_xlabel(r"time $t$")

    return fig, fig2
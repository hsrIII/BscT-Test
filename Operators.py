import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from decimal import Decimal
import ahrs

class Operators:
    def __init__(self, gravity_vector = np.array([0,0,-9.81]), sampling_rate = 0.01):
        self.gravity_vector = gravity_vector
        self.sampling_rate = sampling_rate
        
        self._filter_ekf = ahrs.filters.ekf.EKF(Dt=self.sampling_rate)
        self._filter_ang_rate = ahrs.filters.angular.AngularRate(Dt=self.sampling_rate)
        
        self.w_to_q_methods = ["by hand", "integrator", "EKF"]

    def forwardoperator(self, attitudes, t_list):
        gyr_data = np.zeros((attitudes.shape[0],3))
        acc_data = np.zeros((attitudes.shape[0],3))
        
        for i in range(attitudes.shape[0]):
            q = attitudes[i]
            q_prev = attitudes[i - 1]
            dt = t_list[i] - t_list[i - 1]        
            if i > 0:     # first element in gyr_data is [0,0,0]
                gyr_data[i] = self.q_to_angular_velocity(q_prev, q, dt)
            acc_data[i] = self.q_to_accelerometer_measurement(q)
        
        return gyr_data, acc_data


    def q_to_angular_velocity(self, q1, q2, dt): #https://mariogc.com/post/angular-velocity-quaternions/; q1 = qt, q2 = qt+1
        omega11 = q1[0] * q2[0] + q1[1] * q2[1] + q1[2] * q2[2] + q1[3] * q2[3]
        omega21 = q1[0] * q2[1] - q1[1] * q2[0] - q1[2] * q2[3] + q1[3] * q2[2] 
        omega31 = q1[0] * q2[2] + q1[1] * q2[3] - q1[2] * q2[0] - q1[3] * q2[1] 
        omega41 = q1[0] * q2[3] - q1[1] * q2[2] + q1[2] * q2[1] - q1[3] * q2[0]
        
        w_norm = np.arccos(omega11)*2/dt
        
        if np.sin(w_norm*dt/2) == 0:                    #L'Hospital
            wx = 1/(np.cos(w_norm*dt/2)) *omega21
            wy = 1/(np.cos(w_norm*dt/2)) *omega31
            wz = 1/(np.cos(w_norm*dt/2)) *omega41
        
        else:
            wx = omega21 * w_norm * 1/np.sin(w_norm*dt/2)
            wy = omega31 * w_norm * 1/np.sin(w_norm*dt/2)
            wz = omega41 * w_norm * 1/np.sin(w_norm*dt/2)
        
        angular_velocity = np.array([wx,wy,wz])
        return angular_velocity


    def q_to_accelerometer_measurement(self, q):
        rot_matrix = self.q_to_rotation_matrix(q)
        at = np.linalg.inv(rot_matrix)@self.gravity_vector
        return at


    def simulate_sensor_events(self, gyr_data, acc_data, t_list, method = "by hand"):
            attitudes = np.zeros((gyr_data.shape[0],4))
            timestamps = np.array([])

            for index in range(gyr_data.shape[0]):
                t = t_list[index]
                if index == 0:
                    t_prev = 0
                    q = np.array([1,0,0,0])             #Annahme: q(t=0)=[1,0,0,0]
                    
                else:
                    t_prev = timestamps[-1]

                wt = gyr_data[index,:]
                at = acc_data[index,:]
                
                q, t = self.angular_velocity_to_q(wt, at, q, t, t_prev, method = method)
                timestamps = np.append(timestamps, t)  
                
                attitudes[index,:] = q

            return attitudes, timestamps


    def angular_velocity_to_q(self, wt, at, q, t, t_prev, method = "by hand"):
        if method not in self.w_to_q_methods:
            raise ValueError(f"Method '{method}' is not available. Available methods: {self.w_to_q_methods}.")
        
        if method == self.w_to_q_methods[0]: 
            dt = t-t_prev
            
            wx = wt[0]
            wy = wt[1]
            wz = wt[2]

            w_norm = np.linalg.norm(wt)
            Omega = np.array([[0,-wx,-wy,-wz],[wx,0,wz,-wy],[wy,-wz,0,wx],[wz,wy,-wx,0]])
            Eins = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])

            if w_norm == 0:
                q_res = q

            else:                       
                q_res = (np.cos(w_norm*dt/2)*Eins+(1/w_norm)*np.sin(w_norm*dt/2)*Omega)@q 
                norm_q_res = np.linalg.norm(q_res)
                q_res = q_res/norm_q_res

        if method == self.w_to_q_methods[1]:        #integrator of ahrs, similar method to "by hand"
            q_res = self._filter_ang_rate.update(q, wt, method = "closed")  

        if method == self.w_to_q_methods[2]:        #EKF
            q_res = self._filter_ekf.update(q, wt, at)

        return q_res, t

    def spherecoord_in_q(self, omega, theta, phi): # Bahn auf Kugeloberfläche, umgerechnet zu Attitude (Quaternions)
        ux = np.sin(theta) * np.cos(phi)   
        uy = np.sin(theta) * np.sin(phi)
        uz = np.cos(theta)

        qw = np.cos(omega)
        qi = np.sin(omega) * ux
        qj = np.sin(omega) * uy
        qk = np.sin(omega) * uz
        attitudes = np.array([qw, qi, qj, qk]).T

        return attitudes

    def q_to_rotation_matrix(self, q): #https://automaticaddison.com/how-to-convert-a-quaternion-to-a-rotation-matrix/
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

    def plot_res(self, attitudes, w_, q_res, t_ = None, error = []):
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

        return (fig, axs), (fig2, axs2)

    def plot_comparision(self, att_ref, att_res1, att_res2, t_ref, t_res1 = None, t_res2 = None):
        if t_res1 == None:
            t_res1 = t_ref
        if t_res2 == None:
            t_res2 = t_ref

        fig = plt.figure(figsize=(13,7))
        plt.rcParams['figure.constrained_layout.use'] = True
        gs = GridSpec(4, 3, figure=fig)
        
        ax1 = fig.add_subplot(gs[0, 0])
        ax1.plot(t_ref, att_ref)
        ax1.set_title("Reference Att")
        ax1.grid(True)
        ax2 = fig.add_subplot(gs[0, 1])
        ax2.plot(t_res1, att_res1)
        ax2.set_title("Res Att 1")
        ax2.grid(True)
        ax3 = fig.add_subplot(gs[0, 2])
        ax3.plot(t_res2, att_res2)
        ax3.set_title("Res Att 2")
        ax3.grid(True)

        ax4 = fig.add_subplot(gs[1, :])
        ax4.plot(t_ref, att_ref)
        ax4.plot(t_res1, att_res1)
        ax4.plot(t_res2, att_res2)
        ax4.set_title("Att Ref  Res1  Res2")
        ax4.grid(True)

        ax5 = fig.add_subplot(gs[2, :])
        ax5.plot(t_ref, att_ref-att_res1)
        ax5.set_title("ref-res1")
        ax5.grid(True)

        ax6 = fig.add_subplot(gs[3, :])
        ax6.plot(t_ref, att_ref-att_res2)
        ax6.set_title("ref-res2")
        ax6.grid(True)

        fig.suptitle(f"Comparing two results")

        return fig
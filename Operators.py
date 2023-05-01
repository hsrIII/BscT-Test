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

        self._quiver = None # delete if eulervec does not work

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

        if np.abs(np.sin(w_norm*dt/2)) < 1e-8: #== 0:                    #L'Hospital
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
        #print(f"wt, q: {wt}, {q}")
        if method == self.w_to_q_methods[0]:
            dt = t-t_prev

            wx = wt[0]
            wy = wt[1]
            wz = wt[2]

            w_norm = np.linalg.norm(wt)
            Omega = np.array([[0,-wx,-wy,-wz],[wx,0,wz,-wy],[wy,-wz,0,wx],[wz,wy,-wx,0]])
            Eins = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])

            if w_norm < 1e-8: # ==0
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
        #TODO: produces different results as scipy.spatial.transform.Rotation.from_q(q).as_matrix(), check for errors
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

    def plot_one(self, y_, t_, name=None):
        fig = plt.figure(figsize=(13, 7))
        plt.rcParams['figure.constrained_layout.use'] = True
        plt.title(f"{name}")
        plt.suptitle(f"t[0] = {t_[0]}, t[-1] = {t_[-1]}, t.size = {t_.size}, Average Sampling Rate: (t[-1]-t[0])/t.size = {(t_[-1]-t_[0])/t_.size} s")
        if y_.shape[1]==4:
            plt.plot(t_, y_, label=['qw', 'qi', 'qj', 'qk'], linestyle="-")
            plt.ylabel(r"$\mathbf{q}$")
        if y_.shape[1]==3:
            plt.plot(t_, y_, label=[f'{name}_x', f'{name}_y', f'{name}_z'], linestyle="-")
            plt.ylabel(f"{name} values")
        plt.legend()
        plt.grid(True)
        plt.xlabel(r"time $t$")

        return fig


    def plot_cuboid(self, center, size, q): #https://stackoverflow.com/questions/56332197/rotate-3d-object-with-euler-angles-in-python
        """
       Create a data array for cuboid plotting.
       ============= ================================================
       Argument      Description
       ============= ================================================
       center        center of the cuboid, triple
       size          size of the cuboid, triple, (x_length,y_width,z_height)
       :type size: tuple, numpy.array, list
       :param size: size of the cuboid, triple, (x_length,y_width,z_height)
       :type center: tuple, numpy.array, list
       :param center: center of the cuboid, triple, (x,y,z)
       """

        # suppose axis direction: x: to left; y: to inside; z: to upper
        # get the (left, outside, bottom) point
        ox, oy, oz = center
        l, w, h = size

        # calculating corners
        xco = np.array([ox - l / 2, ox + l / 2])
        yco = np.array([oy - w / 2, oy + w / 2])
        zco = np.array([oz - h / 2, oz + h / 2])

        # corners as vectors
        co1 = np.array([xco[0], yco[0], zco[0]])
        co2 = np.array([xco[1], yco[0], zco[0]])
        co3 = np.array([xco[0], yco[0], zco[1]])
        co4 = np.array([xco[1], yco[0], zco[1]])
        co5 = np.array([xco[0], yco[1], zco[0]])
        co6 = np.array([xco[1], yco[1], zco[0]])
        co7 = np.array([xco[0], yco[1], zco[1]])
        co8 = np.array([xco[1], yco[1], zco[1]])

        # rotate corner vectors with roation matrix calculated from q
        #TODO: find out why scipy.spatial.transform.Rotation.from_quat(q) produces a different rot mat as self.q_to_rotation_matrix(q)
        #r = scipy.spatial.transform.Rotation.from_quat(q)
        #rotmat = r.as_matrix() # different results as q_to_rotation_matrix(), using q_to_rot_mat because of consitency

        rotmatq = self.q_to_rotation_matrix(q)


        co1rot = rotmatq @ co1
        co2rot = rotmatq @ co2
        co3rot = rotmatq @ co3
        co4rot = rotmatq @ co4
        co5rot = rotmatq @ co5
        co6rot = rotmatq @ co6
        co7rot = rotmatq @ co7
        co8rot = rotmatq @ co8

        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')

        #surface in +x direction (when q = [1,0,0,0])
        sx1x = np.array([[co1rot[0], co2rot[0]], [co3rot[0], co4rot[0]]])
        sx1y = np.array([[co1rot[1], co2rot[1]], [co3rot[1], co4rot[1]]])
        sx1z = np.array([[co1rot[2], co2rot[2]], [co3rot[2], co4rot[2]]])

        ax.plot_surface(sx1x, sx1y, sx1z, color='red', rstride=1, cstride=1, alpha=0.6)

        # surface in -x direction
        sx2x = np.array([[co5rot[0], co6rot[0]], [co7rot[0], co8rot[0]]])
        sx2y = np.array([[co5rot[1], co6rot[1]], [co7rot[1], co8rot[1]]])
        sx2z = np.array([[co5rot[2], co6rot[2]], [co7rot[2], co8rot[2]]])

        ax.plot_surface(sx2x, sx2y, sx2z, color='white', rstride=1, cstride=1, alpha=0.6)

        # plot surface in +y direction
        sy1x = np.array([[co2rot[0], co6rot[0]], [co4rot[0], co8rot[0]]])
        sy1y = np.array([[co2rot[1], co6rot[1]], [co4rot[1], co8rot[1]]])
        sy1z = np.array([[co2rot[2], co6rot[2]], [co4rot[2], co8rot[2]]])

        ax.plot_surface(sy1x, sy1y, sy1z, color='blue', rstride=1, cstride=1, alpha=0.6)

        # plot surface in -y direction
        sy2x = np.array([[co1rot[0], co5rot[0]], [co3rot[0], co7rot[0]]])
        sy2y = np.array([[co1rot[1], co5rot[1]], [co3rot[1], co7rot[1]]])
        sy2z = np.array([[co1rot[2], co5rot[2]], [co3rot[2], co7rot[2]]])

        ax.plot_surface(sy2x, sy2y, sy2z, color='black', rstride=1, cstride=1, alpha=0.6)

        # plot surface in +z direction
        sz1x = np.array([[co3rot[0], co4rot[0]], [co7rot[0], co8rot[0]]])
        sz1y = np.array([[co3rot[1], co4rot[1]], [co7rot[1], co8rot[1]]])
        sz1z = np.array([[co3rot[2], co4rot[2]], [co7rot[2], co8rot[2]]])

        ax.plot_surface(sz1x, sz1y, sz1z, color='green', rstride=1, cstride=1, alpha=0.6)

        # plot surface in -z direction
        sz2x = np.array([[co1rot[0], co2rot[0]], [co5rot[0], co6rot[0]]])
        sz2y = np.array([[co1rot[1], co2rot[1]], [co5rot[1], co6rot[1]]])
        sz2z = np.array([[co1rot[2], co2rot[2]], [co5rot[2], co6rot[2]]])

        ax.plot_surface(sz2x, sz2y, sz2z, color='pink', rstride=1, cstride=1, alpha=0.6)

        ax.axes.set_xlim3d(left=-2.5, right=2.5)
        ax.axes.set_ylim3d(bottom=-2.5, top=2.5)
        ax.axes.set_zlim3d(bottom=-2.5, top=2.5)

        ## Add title
        plt.title('Plot Title', fontsize=20)

        ##labelling the axes
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

        return fig

    def animate_attitudes(self, attitudes, t_, dt = 0.5):
        t0 = 0
        for i, q in enumerate(attitudes):
            t = t_[i]
            if t - t0 > dt:
                self.plot_cuboid((0, 0, 0), (1, 2, 0.5), q)
                plt.show(block=False)
                plt.pause(0.000001)
                plt.close()
                t0 = t


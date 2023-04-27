import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import Operators as _qOp
import Testdata


# Reference Attitude
print(f"++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 01 reference attitude")
attitudes, t_ = Testdata.orientation_testcases(case = "E")

# Sensor Data
print(f"++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 02 Sensor data from reference attitude")
acc_data = _qOp.attitudes_to_acc_data(attitudes)        # Accelerometer Data
gyr_data, t_ = _qOp.forwardoperator(attitudes, t_)      # Gyroscope Data

'''
# Add Noise
print(f"++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++  02b Add Noise")
mu, sigma = 0, 0.001 # bias, standard deviation
gyr_data += np.random.normal(mu, sigma, gyr_data.shape)
t_ += np.random.normal(0, 0.01/10, t_.shape)
'''

# Integration of Forwardoperator-Data
print(f"++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 03 attitude from angular rate")
att_res, t_ = _qOp.simulate_sensor_events(gyr_data, acc_data, t_, method = "integrator")     #resulting Attitude
#att_res2, t2 = _qOp.simulate_sensor_events(gyr_data, t_, mode = "ahrs") # does not work

# Using a filter
'''
ekf = ahrs.filters.ekf.EKF(Dt=0.01)
att_fil = np.zeros_like(att_res)
att_fil[0,:] = np.array([1,0,0,0])

for t in range(1,att_fil.shape[0]):
    att_fil[t,:] = ekf.update(att_fil[t-1], gyr_data[t], acc_data[t])
'''

#att_fil, t2 = _qOp.simulate_sensor_events(gyr_data, acc_data, t_, method = "EKF")

# Plotting
print(f"++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 04 plotting")
error = attitudes - att_res
plots = _qOp.plot_res(attitudes, gyr_data, att_res, t_, error)
comp = _qOp.plot_comparision(attitudes, att_res, att_fil, t_)

if input("plot? y, n: ") == "y":
    plt.show()

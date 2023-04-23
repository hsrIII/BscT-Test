import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import Operators as _qOp
import Testdata

# Reference Attitude
print(f"++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 01 reference attitude")
attitudes, t_ = Testdata.orientation_testcases(case = "G")

# Sensor Data
print(f"++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 02 Sensor data from reference attitude")
acc_data = _qOp.attitudes_to_acc_data(attitudes)        # Accelerometer Data
gyr_data, t_ = _qOp.forwardoperator(attitudes, t_)      # Gyroscope Data

'''
# Add Noise
mu, sigma = 0, 0.001 # bias, standard deviation
gyr_data += np.random.normal(mu, sigma, gyr_data.shape)
t_ += np.random.normal(0, step/10, t_.shape)
'''

# Integration of Forwardoperator-Data
print(f"++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 03 attitude from angular rate")
att_res, t_ = _qOp.simulate_sensor_events(gyr_data, t_)     #resulting Attitude

# Plotting
print(f"++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 04 plotting")
error = attitudes - att_res
plots = _qOp.plot_res(attitudes, gyr_data, att_res, t_, error)

if input("plot? y, n: ") == "y":
    plt.show()

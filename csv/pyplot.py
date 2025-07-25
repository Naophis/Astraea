from multiprocessing import dummy
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from scipy.optimize import curve_fit

fig = plt.figure()
raw = 9
L90 = fig.add_subplot(1, raw, 1)
L45_3 = fig.add_subplot(1, raw, 2)
L45_2 = fig.add_subplot(1, raw, 3)
L45 = fig.add_subplot(1, raw, 4)
F = fig.add_subplot(1, raw, 5)
R45 = fig.add_subplot(1, raw, 6)
R45_2 = fig.add_subplot(1, raw, 7)
R45_3 = fig.add_subplot(1, raw, 8)
R90 = fig.add_subplot(1, raw, 9)


def nonlinear_fit(x, a, b):
    return a/np.log(x) - b


def main_plot(fig, file, col_idx, str):

    df = pd.read_csv(file)
    input_csv = df.sort_values('dist', ascending=False)
    dist = input_csv[input_csv.keys()[0]]
    sensor_raw = input_csv[input_csv.keys()[col_idx]]  # csvの何行目のデータを使うか

    param, cov = curve_fit(nonlinear_fit, sensor_raw, dist)
    print("{}: [{}, {}] #cov[{},{}]" .format(str, param[0], param[1],cov[0][0], cov[1][1]))
    min_x = sensor_raw.min()
    max_x = sensor_raw.max()
    x = np.linspace(min_x, max_x, 20)
    y = nonlinear_fit(x, param[0], param[1])

    fig.scatter(sensor_raw, dist)
    fig.plot(x, y, label='3')

    return param


res_l45_3 = main_plot(L45_3, "./result_l.csv", 2, "L45_3")
res_l45_2 = main_plot(L45_2, "./result_l.csv", 3, "L45_2")
res_l45 = main_plot(L45, "./result_l.csv", 4, "L45")
res_r45 = main_plot(R45, "./result_r.csv", 6, "R45")
res_r45_2 = main_plot(R45_2, "./result_r.csv", 7, "R45_2")
res_r45_3 = main_plot(R45_3, "./result_r.csv", 8, "R45_3")

res_f = main_plot(F, "./result_f.csv", 5, "F")
res_l90_near = main_plot(L90, "./result_f.csv", 1, "L90_near")
res_r90_near = main_plot(R90, "./result_f.csv", 9, "R90_near")

res_l90_mid = main_plot(L90, "./result_f.csv", 1, "L90_mid")
res_r90_mid = main_plot(R90, "./result_f.csv", 9, "R90_mid")

res_l90_far = main_plot(L90, "./result_f.csv", 1, "L90_far")
res_r90_far = main_plot(R90, "./result_f.csv", 9, "R90_far")

plt.show()

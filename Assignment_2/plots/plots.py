import matplotlib.pyplot as plt
import numpy as np
import math

C3_txt_path = 'C3 - data.txt'
# t, x0, x1, theta, xd0, xd1, e0, e1, tau1, tau2, tau3

C4_txt_path = 'C4 - data.txt'
# t, beta

type = "C4"


def main():
    if type == "C3":
        C3_plot()
    elif type == "C4":
        C4_plot()


def C3_plot():
    data = np.loadtxt(C3_txt_path)
    # plot of x0 & xd0
    fig, ax1 = plt.subplots()
    ax1.plot(data[:, 0], data[:, 1], 'r.')
    ax1.set_xlabel('time')

    ax1.set_ylabel('x0', color='r')
    ax1.tick_params('y', colors='r')

    ax2 = ax1.twinx()
    ax2.plot(data[:, 0], data[:, 4], 'b-')
    ax2.set_ylabel('xd0', color='b')
    ax2.tick_params('y', colors='b')

    fig.tight_layout()
    plt.show()
    # plot of x1 & xd1
    fig, ax1 = plt.subplots()
    ax1.plot(data[:, 0], data[:, 2], 'r.')
    ax1.set_xlabel('time')

    ax1.set_ylabel('x0', color='r')
    ax1.tick_params('y', colors='r')

    ax2 = ax1.twinx()
    ax2.plot(data[:, 0], data[:, 5], 'b-')
    ax2.set_ylabel('xd0', color='b')
    ax2.tick_params('y', colors='b')

    fig.tight_layout()
    plt.show()
    # plot of theta
    fig, ax1 = plt.subplots()
    ax1.plot(data[:, 0], data[:, 3], 'b-')
    ax1.set_xlabel('time')

    ax1.set_ylabel('theta', color='b')
    ax1.tick_params('y', colors='b')

    fig.tight_layout()
    plt.show()
    # plot of e0
    fig, ax1 = plt.subplots()
    ax1.plot(data[:, 0], data[:, 6], 'b-')
    ax1.set_xlabel('time')

    ax1.set_ylabel('error0', color='b')
    ax1.tick_params('y', colors='b')

    fig.tight_layout()
    plt.show()
    # plot of e1
    fig, ax1 = plt.subplots()
    ax1.plot(data[:, 0], data[:, 7], 'b-')
    ax1.set_xlabel('time')

    ax1.set_ylabel('error1', color='b')
    ax1.tick_params('y', colors='b')

    fig.tight_layout()
    plt.show()
    # plot of tau1
    fig, ax1 = plt.subplots()
    ax1.plot(data[:, 0], data[:, 8], 'b-')
    ax1.set_xlabel('time')

    ax1.set_ylabel('tau1', color='b')
    ax1.tick_params('y', colors='b')

    fig.tight_layout()
    plt.show()
    # plot of tau2
    fig, ax1 = plt.subplots()
    ax1.plot(data[:, 0], data[:, 9], 'b-')
    ax1.set_xlabel('time')

    ax1.set_ylabel('tau2', color='b')
    ax1.tick_params('y', colors='b')

    fig.tight_layout()
    plt.show()
    # plot of tau3
    fig, ax1 = plt.subplots()
    ax1.plot(data[:, 0], data[:, 10], 'b-')
    ax1.set_xlabel('time')

    ax1.set_ylabel('tau3', color='b')
    ax1.tick_params('y', colors='b')

    fig.tight_layout()
    plt.show()


def C4_plot():
    data = np.loadtxt(C4_txt_path)
    # plot of angle beta & angular velocity
    angle = []
    angular_vel = []
    offset = 0

    for i in range(len(data[:, 1])):
        temp = math.atan(((data[i, 1] - 0.45) / (data[i, 2] - 0.6)))
        if i > 0:
            if math.atan(((data[i-1, 1] - 0.45) / (data[i-1, 2] - 0.6)))*temp < 0 :
                offset = angle[i-1]
            if ((data[i, 1] - 0.45) > 0) and ((data[i, 2] - 0.6) < 0):
                temp = math.atan((data[i, 1] - 0.45)/abs(data[i, 2] - 0.6))
            if ((data[i, 1] - 0.45) < 0) and ((data[i, 2] - 0.6) < 0):
                temp = math.atan((data[i, 2] - 0.6)/(data[i, 1] - 0.45))
            if ((data[i, 1] - 0.45) < 0) and ((data[i, 2] - 0.6) > 0):
                temp = math.atan((abs(data[i, 1] - 0.45) / (data[i, 2] - 0.6)))
            if ((data[i, 1] - 0.45) > 0) and ((data[i, 2] - 0.6) > 0):
                temp = math.atan((data[i, 2] - 0.6)/(data[i, 1] - 0.45))
        angle.append(temp + offset)
    for i in range(len(angle)):
        angle[i] = angle[i] - 1.57078 # pi/2

    for i in range(len(angle) - 1):
        angular_vel.append((angle[i + 1] - angle[i]) / (data[i + 1, 0] - data[i, 0]))

    fig, ax = plt.subplots(2, 1)
    ax[0].plot(data[1:, 0], angle[1:], color = 'C0')
    ax[0].set_xlabel('time')

    ax[0].set_ylabel('angle', color='r')
    ax[0].tick_params('y', colors='r')
    ax[0].grid

    ax[1].plot(data[2:len(data[:, 1]), 0], angular_vel[1:], color = 'C1')
    ax[1].set_ylabel('angle velocity', color='b')
    ax[1].tick_params('y', colors='b')
    ax[1].set_yticks([-2, 0, 2, 4])
    ax[1].grid

    fig.tight_layout()
    plt.show()


if __name__ == '__main__':
    main()

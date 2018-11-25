import pandas as pd
import seaborn as sns
import numpy as np
import matplotlib.pyplot as plt

sns.set(style='ticks', palette='Set2')


def compute_spline(start_pos, final_pos, start_velocity, final_velocity, start_time, final_time):
    stepsize = 0.01
    t = np.arange(start_time, final_time+stepsize, stepsize)

    # cubic spline parameters
    a0 = start_pos
    a1 = start_velocity
    a2 = 3/((final_time-start_time)**2) * (final_pos-start_pos) - 2/(final_time-start_time) * start_velocity - 1/(final_time-start_time) * final_velocity
    a3 = -2/((final_time-start_time)**3) * (final_pos-start_pos) + 1/((final_time-start_time)**2) * (start_velocity + final_velocity)

    print(a0, a1, a2, a3);

    # cubic spline
    f = a0 + a1*(t-start_time) + a2*(t-start_time)**2 + a3*(t-start_time)**3

    return t, f


def velocity_heuristic(start, via, end):
    if start <= via <= end or start >= via >= end:
        return 0.5*((via-start)/2.5 + (end-via)/2.5)
    else:
        return 0

if __name__ == "__main__":
    fig, ax = plt.subplots(3, 1, figsize=(10,14))
    fig.tight_layout(pad=5)

    start = [0, 0, 0]
    via = [-np.pi/4, np.pi/2, 0]
    end = [-np.pi/2, np.pi/4, 0]

    for i in range(3):
        via_vel = velocity_heuristic(start[i], via[i], end[i])

        t1, f1 = compute_spline(start[i], via[i], 0, via_vel, 0, 2.5)
        t2, f2 = compute_spline(via[i], end[i], via_vel, 0, 2.5, 5.0)

        sns.lineplot(t1, f1, color="C0", ax=ax[i])
        sns.lineplot(t2, f2, color="C1", ax=ax[i])

        dotsize = 100
        ax[i].scatter(0.0, start[i], s=dotsize, zorder=3)
        ax[i].scatter(2.5, via[i], s=dotsize, zorder=3)
        ax[i].scatter(5.0, end[i], s=dotsize, zorder=3)

        ax[i].set_title("trajectory in joint space")
        ax[i].set_xlabel("timestep t")
        ax[i].set_ylabel("joint parameter q_" + str(i+1))
        ax[i].grid()

    ax[0].set_yticks([0, -np.pi/4, -np.pi/2, -0.5, -1.0, -1.5])
    ax[0].set_yticklabels(["0(start)", "-pi/4(via)", "-pi/2(end)", "-0.5", "-1.0", "-1.5"])
    ax[1].set_yticks([0, np.pi/2, np.pi/4, 0.5, 1.0, 1.5])
    ax[1].set_yticklabels(["0(start)", "pi/2(via)", "pi/4(end)", "0.5", "1.0", "1.5"])

    plt.show()

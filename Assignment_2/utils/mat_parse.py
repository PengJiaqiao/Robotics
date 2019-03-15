import pandas as pd
import numpy as np
import sys


def parse(infile, outfile):
    df = pd.read_csv(infile, sep="\t")
    df = df.dropna(axis='columns', how='all')

    if len(df.columns) != 22: return

    names = [
        "time",                     # sec
        "q1", "q2", "q3",           # rad
        "dq1", "dq2", "dq3",        # rad/sec
        "qd1", "qd2", "qd3",        # rad
        "tau1", "tau2", "tau3",     # newton-meter
        "x1", "x2", "x3",
        "dx1", "dx2", "dx3",
        "xd1", "xd2", "xd3"
    ]
    df.columns = names

    # normalize timestamps
    start_time = df["time"][0]
    df["time"] -= start_time

    # convert rad- to deg-values
    for c in names[1:10]:
        df[c] = np.rad2deg(df[c])

    df.to_csv(outfile, decimal = ",", sep=";", line_terminator=";\n", header=False, index=False)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        parse(sys.argv[1], sys.argv[2])
    else:
        print("usage: python %s <infile> <outfile>" % sys.argv[0])

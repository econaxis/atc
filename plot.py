import pandas as pd
import matplotlib.pyplot as plt
import matplotlib
import functools

pd.set_option('display.max_colwidth', None)
pd.set_option('display.max_columns', None)

df = pd.read_csv("output.txt", index_col=False)
df.columns.values[0] = "x"
df.columns.values[1] = "y"
df.columns.values[2] = "time"
df.columns.values[3] = "id"
fig, ax = plt.subplots(1, 1)
scat = ax.scatter(df.iloc[:, 0], df.iloc[:, 1], c=df.iloc[:, 2])


@functools.cache
def planes_in_column(xval):
    # Get all the data points in the range near xval
    return df[df["x"].between(xval - 6, xval + 6)]


@functools.cache
def planes_in_row(yval):
    return df[df["y"].between(yval - 6, yval + 6)]


# Verifier
# Loops through all coordinate positions and checks for overlapping planes
for x in range(0, 1000):
    for y in range(0, 1000):
        xf = planes_in_column(x)
        yf = planes_in_row(y)
        at_square = xf.merge(yf, how='inner')
        if at_square.shape[0] > 1:
            at_square = at_square.drop_duplicates(subset=["id"])
            if at_square.shape[0] > 1:
                diff = at_square.sort_values("time").diff().iloc[1:, :]["time"]
                if not (diff > 1.5).all():
                    # Might be false positives
                    print("Possible error candidate: \n", at_square)
    print(x)

print("Done loading and checking")


def plot_func(id):
    print(id)
    cur = df[df.iloc[:, 2].between(id - 2, id + 2)]
    cur = cur.drop_duplicates(subset=["id"])
    scat.set_offsets(cur.iloc[:, 0:2])
    scat.set_sizes([2] * df.shape[0])
    return scat,


anim = matplotlib.animation.FuncAnimation(fig, plot_func, frames=range(0, 300), interval=50, blit=True)
writergif = matplotlib.animation.PillowWriter(fps=29)
anim.save("atc.gif", writer=writergif)

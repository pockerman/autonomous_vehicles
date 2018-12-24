import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class SimpleAnimator(object):

    def __init__(self):
        self.fig, self.ax = plt.subplots()
        self.xdata = []
        self.ydata = []
        self.ln, = plt.plot([], [], 'ro', animated=True)

    def init_animation(self):
        self.ax.set_xlim(0, 20)
        self.ax.set_ylim(0, 20)
        return self.ln,

    def update_animation(self, frame):
        self.xdata.append(frame[0])
        self.ydata.append(frame[1])  # np.sin(frame))
        self.ln.set_data(self.xdata, self.ydata)
        return self.ln,

    def animate(self, frames):
        ani = FuncAnimation(self.fig, self.update_animation, frames=frames,
                            init_func=self.init_animation, blit=True)
        plt.show()
import matplotlib.pyplot as plt

class TwoDPlotter(object):

    def __init__(self, **kwargs):

        self.__plot_title = "No Title"
        if 'plot_title' in kwargs.keys():
            self.__plot_title = kwargs['plot_title']

        self.__y_axis_title = "No y-axis Title"
        if 'ylabel' in kwargs.keys():
            self.__y_axis_title = kwargs['ylabel']

        self.__x_axis_title = "No x-axis Title"
        if 'xlabel' in kwargs.keys():
            self.__x_axis_title = kwargs['xlabel']


    @property
    def xlabel(self):
        return self.__x_axis_title

    @xlabel.setter
    def xlabel(self, value):
        self.__x_axis_title = value

    @property
    def ylabel(self):
        return self.__y_axis_title

    @ylabel.setter
    def ylabel(self, value):
        self.__y_axis_title = value

    def plot(self, x, y, label=None):
        plt.plot(x,y, label=label)


    def show_plots(self):
        plt.ylabel(self.__y_axis_title)
        plt.xlabel(self.__x_axis_title)

        if self.__plot_title != "No Title":
            plt.title(self.__plot_title)
        plt.grid(True)
        plt.legend()
        plt.show()

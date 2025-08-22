import matplotlib
from matplotlib.lines import Line2D

matplotlib.use("QtAgg")
import matplotlib.pyplot as plt
from collections import deque


class LivePlotter:
    """
    A real time plotter
    """

    def __init__(
        self,
        root_dir: str,
        y_fields: tuple,
        plot_name: str,
        plot_size: int,
        x_label: str,
        y_label: str,
        title: str,
        flip_axis: bool = False,
    ):
        """
        Initializes the LivePlotter and sets up the interactive plot window

        Args:
            root_dir (str): Directory where the final plot will be saved
            y_fields (tuple[tuple[str, str]]): Tuples of (line style, label) for each data series
            plot_name (str): Filename for the saved plot
            plot_size (int): Number of points to keep in the plot at any one time
            x_label (str): Label to display on the x-axis
            y_label (str): Label to display on the y-axis
            title (str): Plot title
            flip_axis (bool): If True, swaps x and y data for horizontal plotting
        """
        plt.ion()  # Turn on interactive mode for live updates
        self.flip_axis = flip_axis
        self.save_dir = f"{root_dir}/results"
        self.plot_name = plot_name

        self.fig, self.ax = plt.subplots(figsize=(12, 6))
        self.ax.set_xlabel(y_label if flip_axis else x_label)
        self.ax.set_ylabel(x_label if flip_axis else y_label)
        self.ax.set_title(title)
        self.ax.grid(True)

        # Create line for each y field
        self.lines: list[Line2D] = [
            self.ax.plot([], [], line_style, label=label)[0]
            for line_style, label in y_fields
        ]

        self.ax.legend()

        # Buffers for x and y data (fixed length sliding windows)
        self.y_data = [deque(maxlen=plot_size) for _ in y_fields]
        self.x_data = deque(maxlen=plot_size)

        self.fig.tight_layout()
        plt.show()

    def update(self, new_y_data: tuple, new_x_data):
        """
        Updates the live plot with new data points

        Args:
            new_y_data (tuple): A tuple of values (one for each y-field)
            new_x_data (Any): A new x-value
        """
        self.x_data.append(new_x_data)

        for i, y_data in enumerate(self.y_data):
            y_data.append(new_y_data[i])

            # Update line data depending on axis orientation
            if self.flip_axis:
                self.lines[i].set_data(y_data, self.x_data)
            else:
                self.lines[i].set_data(self.x_data, y_data)

        # Recalculates plot bounds and refresh canvas
        self.ax.relim()
        self.ax.autoscale_view()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def close(self):
        """
        Saves the final plot and closes the figure window
        """
        self.fig.savefig(f"{self.save_dir}/{self.plot_name}", dpi=300)
        print(f"Saved final plot to {self.save_dir}")
        plt.ioff()
        plt.close(self.fig)
        print("Plot window closed\n")

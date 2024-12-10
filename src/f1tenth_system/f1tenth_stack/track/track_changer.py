import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Button

# Load data from CSV
data = np.loadtxt(
    '/home/nvidia/ros2_ws/src/adaptive_pure_pursuit/timed.csv', delimiter=",")
x, y = data[:, 0], data[:, 1]

# Create figure and scatter plot
fig, ax = plt.subplots()
sc = ax.scatter(x, y, color="blue", picker=True)  # Enable picking
ax.set_xlabel("X")
ax.set_ylabel("Y")
plt.title("Drag and Drop Point Adjustment")

selected_index = None  # To store the index of the selected point
dragging_point = False  # To track dragging state

# Event handler for mouse button press


def on_press(event):
    global selected_index, dragging_point
    # Check if a point is clicked
    if event.inaxes != ax:
        return
    # Calculate the closest point to the click
    distances = np.sqrt((x - event.xdata)**2 + (y - event.ydata)**2)
    selected_index = np.argmin(distances)
    # Set dragging state to true if the click is close to a point
    if distances[selected_index] < 0.1:  # Set a small threshold for selection
        dragging_point = True

# Event handler for mouse motion


def on_motion(event):
    global dragging_point
    if dragging_point and selected_index is not None:
        # Update the position of the selected point
        x[selected_index] = event.xdata
        y[selected_index] = event.ydata
        sc.set_offsets(np.c_[x, y])
        fig.canvas.draw_idle()

# Event handler for mouse button release


def on_release(event):
    global dragging_point
    dragging_point = False


# Connect events to the figure
fig.canvas.mpl_connect("button_press_event", on_press)
fig.canvas.mpl_connect("motion_notify_event", on_motion)
fig.canvas.mpl_connect("button_release_event", on_release)

# Save button
save_ax = plt.axes([0.8, 0.025, 0.1, 0.04])
save_button = Button(
    save_ax, 'Save', color='lightgoldenrodyellow', hovercolor='0.975')


def save(event):
    np.savetxt("/home/nvidia/Downloads/sahil_mc_raceline.csv", np.c_[x, y], delimiter=",", fmt="%.7f")
    print("Points saved to updated_points.csv")


save_button.on_clicked(save)

plt.show()

# Import necessary libraries
from HAL import HAL
from GUI import GUI
from MAP import MAP

import numpy as np

# Set the robot's speed and angular velocity
HAL.setV(0.5)
HAL.setW(0.5)

# Get the robot's current pose (position and orientation)
pose = HAL.getPose3d()
x, y, theta = pose.x, pose.y, pose.yaw

# Get the target pose (destination coordinates) from the GUI
target_x, target_y = GUI.getTargetPose()

# Get the MAP image from the MAP class
MAP_image = MAP.getMap()

# Calculate the gradient field using the distance from each point in the MAP to the target
gradient_field = np.zeros_like(MAP_image)
for i in range(MAP_image.shape[0]):
  for j in range(MAP_image.shape[1]):
    # If the current cell is an obstacle, assign it a value of 0
    if MAP_image[i, j] == 0:
      gradient_field[i, j] = 0
    else:
      # Calculate the distance from the current cell to the target
      distance = np.sqrt((i - target_x)**2 + (j - target_y)**2)
      # Assign the value of the distance to the gradient field cell
      gradient_field[i, j] = distance
# Show the gradient field on the GUI
GUI.showNumpy(gradient_field)

# Use the gradient field to GUIde the robot's movement towards the target
while True:
    # Get the robot's current pose
    x, y, theta = HAL.getPose3d()
    
    # Calculate the robot's next step using the gradient field
    current_cell = MAP.rowColumn([x, y])
    next_step = np.argmax(gradient_field[current_cell[0], current_cell[1]])
    
    # Move the robot to the next step
    HAL.setV(next_step[0])
    HAL.setW(next_step[1])
    
    # Show the updated gradient field on the GUI
    GUI.showNumpy(gradient_field)
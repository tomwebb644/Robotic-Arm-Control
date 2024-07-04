
Scope: The goal of this project was to create code that could communicate with my computer vision code, sending movement instructions to a robotic arm though the use of MoveIt Commander instructions. As the previous project could already identify shapes, the work would be converting locations in the image, to locations in a 3-dimensional space that the arm could understand.
Action: The first step was to adapt the code from the computer vision to give pixel locations and size of object. Once these were gathered, a transform could be applied to convert the pixel locations in the image to the real-life locations that could then be given to the MoveIt commander code, facilitating the movement of the robotic arm.
Result: Some issues surfaced when using movement commands originally, ensuring that the arm knows what the bounds of its operation are, including defining any fixtures within its reach. Ensuring all parameters are set correctly prevented collisions though poorly generated paths for the arm.


Box identifier with pick and place motion programmed through MoveIt!

This Folder should contain 6 files: 'Coordinate_Publisher.py', 'Move_Arm_with_GUI.py', 
'Move_Arm_with_text.py', 'Assessment 3 Recording 1.mp4', 'Analysis Report.docx' and this readme file.

To run this code you must have OpenCV, rospy and moveit python libraries installed.
You must also have ur5_gazebo_simple and demo_moveit_config packages available in your workspace.
Finally, you must have gazebo 7.16.1 or later installed


This codes function is to ask the user to select an object in a ur5 robotic arms reach, pick it up, 
and move it to another location.


To run the code, open a terminal window and run the follwing command:
roslaunch ur5_gazebo_simple ur5_gazebo_nogrip.launch

Then open an additional tab or window and run:
roslaunch ur5_gazebo_simple spawn_cubes.launch

followed by (in the same terminal):
roslaunch demo_moveit_config demo_planning_execution_headless.launch

In a another new tab/window, cd into the folder containing the scripts, 
e.g. "cd assessment1_ws/src/ur5_gazebo_simple/scripts/Auxillary"

Then run:
python Coordinate_Publisher.py

In a separate tab/window, then run:
python Move_Arm_with_GUI.py

When this last script has been excecuted, after a short delay,
the arm will move to an initial position.

You will then be shown an image from the camera located above the table, 
click on the box in the preview that you would like to pick up and press any key.

The arm will move to, "pick up cube" and then move back to its initial position 
and you will be promped to select another box.

To exit the code, make the terminal running the 'Move_Arm_with_GUI.py' script and press ctrl+C.

Alternatively, you can run 'Move_Arm_with_text.py' in place of 'Move_Arm_with_GUI.py' 
to choose the box by colour and index inside the terminal.

Recording of this project functioning can be found here: https://drive.google.com/file/d/1gWnTE1fnhlGcHRu585OcNONnjqEIQUgL/view?usp=sharing
Analysis report can be found here: https://drive.google.com/file/d/1bUi7znbMS6iRDTXdLj3wylXP6avQvHpa/view?usp=drive_link


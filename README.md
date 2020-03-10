# proj2_python
Dijkstra algorithm for point and rigid robot

Implemented Dijkstra algorithm for point and rigid robot.
Implemented in python.

required numpy, math, cv2 libraries.

Half plane equation and semi-algebric models are used to define obstacle space. Minkowski sum is used to expand the map for rigid robot.
The robot can move as 8 connected space, up, down, left , right, and diagonally between them. The diagonal movement costs √2 and other movements are considered 1.

Node exploration and optimal path plot is done using opencv.

The implementation is done into two files namely, Dijkstra_point.py and Dijkstra_rigid.py for point and rigid robots respectively.

If using an IDE, just open the attached .py files and run them

for the terminal, go to the location of the .py files, and run them with python filename.py (respective stored filenames)

The user will be asked to set the start and end point coordinates and resolution for point robot, for rigid robot additional of radius and clearance needs to be entered by the user.

The animation of the map will be generated.

The respective plot will also be stored in the same folder as .py files.



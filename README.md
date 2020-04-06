# baxter-mobility-base-simdemo
ROS Sim for Baxter + mobility base in VXLab

https://youtu.be/U0TmnjHC2r8

Requires install of docker-ce on your platform to build and run container.

Experimental navigation stack on "navigation" branch.

Build config:

(optionally, set 02proxy to point to a nearby apt-cacher-ng proxy)

Build using:

./build

Run using:

./run

This will start three containers: Simulation platform (vxlab-rosie) in the foreground. In the background: X session for gazebo (novnc), X session for rviz (rviz).

In simulation platform:

./simstart

To see the gazebo window, run a browser on the same machine, with the URL: http://localhost:8081/vnc_auto.html

Once the gazebo world has started up, very soon you will should lift the robot arms off the floor fairly soon, or the arms touching the ground plane triggers a bug which tips the robot over: Use CTRL-Z, then type "bg" + ENTER, to put the script in the background. Then type:

./lift-arms

To test navigation (experimental, including launching an rviz window):

./navstart

To see the rviz window, run a browser on the same machine, with the URL: http://localhost:8082/vnc_auto.html

Using rviz requires reading some documentation. The Displays pane on the left hand side has an "Add" button which is just out of view off the bottom of the screen. You can drag to detach the Displays pane and move it somewhere more convenient.

Press the Add button and explore adding different displays. The key ones are "Map" (by topic) and "Robot model" (by display type). Once these two are added you can also click on "2D Nav Goal", then click on the map to position an arrow for the desired location of the robot. Amusingly, since the map provided is of the actual VXLab, but does not match the simulated environment, Rosie does a plausible job at navigating to a given position, but does not succeed.

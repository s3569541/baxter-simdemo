# baxter-mobility-base-simdemo (blue-mir100 branch)
## ROS Simulation for two Baxter's in two VXLab

## Features:
- Video demonstration available: https://youtu.be/O3mMtT-RYps
- Finate report on project available in the root directory of this repository: ~/robocostack-final-report.pdf
- Communication from a scanning Baxter to a stacking Baxter for a set of ALVAR marker blocks using ROS Topics
- Computer vision for Baxter (Alvar)
- Collection of demo scripts mounted under ~/rosie/moveit on blue-mir100 branch

## Prerequisites:
- docker-ce
- docker-compose
- optionally, edit 02proxy to point to a nearby Debian apt-cacher-ng proxy

## Build using:

docker-compose build

## Run:
First time only:
./create-vxlab-network

`docker-compose -f docker-compose-two.yml up` (to have two gazebo simulations side by side)

This will start several containers in the background:
- gazebo and gazebo2  (Simulator "gzserver" and core assets for robots)
- novnc, display2 (X sessions for graphical output in browser)
- alvar-head (Marker recognition for Rosie's head camera)

To stop:

`docker-compose down -d`

## View simulation:

Point your browser on the simulation host, substituting HOSTNAME: http://HOSTNAME:8080/vnc_auto.html

To view output:

Point your browser at HOSTNAME:8080 (main output on "novnc" display) or HOSTNAME:8081 (secondary output on "display2")

You may need to run the gazebo client manually in the master container if it does not start automatically; see below.

## Scan/Stack demos:

Scanning Baxter (gazebo container):
		
	docker exec -it gazebo bash
	DISPLAY=novnc:0 gzclient 	   (view the simulated world)

	docker exec -it gazebo bash
	DISPLAY=novnc:0 rviz 		   (see Baxters view of the world)

	docker exec -it gazebo bash
	cd moveit && source ./init 	   (startup the moveIt framework)

	./set_arms_to_scan.py	init       (on start up only: moves left arm out of the way and opens grippers)
	./set_arms_to_scan.py yaw    	   (set scanning arm to ideal position)
	./spawnstack 		           (uncomment only yaw instructions before execution)
	./blockstack_alvar_demo.py primary (scans blocks and publishes result)


Stacking Baxter (gazebo2 container):

	docker exec -it gazebo2 bash
	DISPLAY=novnc:0 gzclient 	  (view the simulated world)

	docker exec -it gazebo2 bash
	DISPLAY=novnc:0 rviz 		  (see Baxters view of the world)

	docker exec -it gazebo2 bash
	cd moveit && source ./init	  (startup the moveIt framework)
	../twin-relay 			  (communication from gazebo to gazebo2)

	docker exec -it gazebo2 bash
	cd moveit && source ./init 	  (startup the moveIt framework)
	./set_arms_to_scan.py 		  (on start up only: moves right arm out of the way)
	./spawnpickup 		          (sets pickup blocks to ideal position)
	./blockstack_alvar_demo.py stack  (stacks blocks based on received data)
	

When finished: 
  
  	docker-compose down --remove-orphans


## Debug (Rviz)

Refer to documentation for Rviz. The Displays pane on the left hand side has an "Add" button which is just out of view off the bottom of the screen. You can drag to detach the Displays pane and move it somewhere more convenient.

Press the Add button and explore adding different displays. The key ones are "Map" (by topic) and "Robot model" (by display type). Once these two are added you can also click on "2D Nav Goal", then click on the map to position an arrow for the desired location of the robot. Refer to ROS documentation for navigation.


## Notes:

- On startup, especially first startup, you may need to interrupt and restart a graphical application such as gzclient a few times before it first draws correctly. This may include a stream of error messages such as:
`QXcbConnection: XCB error: 2 (BadValue), sequence: 1555, resource id: 1200, major code: 130 (Unknown), minor code: 3`
Some improvements are suggested at: http://wiki.ros.org/docker/Tutorials/GUI

- Several containers mount a docker volume under "~/rosie". Thus, changes to this directory are persistent and cause changes to the directory with the same name on the container host. Be careful! Take backups!

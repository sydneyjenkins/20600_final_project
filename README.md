# Final Project

**Team**: Kir Nagaitsev, Rachel Wang, Sydney Jenkins, Alec Blagg

**Demo**: https://youtu.be/IbbQhGGPak8

## Project Description
Our project is a predator-prey simulation in which there are multiple “prey” robots and a single “predator” robot. The three prey robots each exhibit a specific behavior that helps them avoid the predator robot. The prey behaviors include: (1) rotating in place until it sees the predator, in which case it drives backwards away from the predator, (2) finding and following the nearest wall, and (3) finding and following around the nearest cylindrical obstacle. The predator robot was trained to catch the prey using a genetic algorithm. The genetic algorithm optimized a small set of predator parameters using a significant generation size and repeated training runs for each set of parameters to determine predator fitness. The robots all move within an enclosed area with three cylindrical obstacles, and their initial locations for each generation are randomly selected from a predetermined set of possible locations. We were interested in this project because we wanted to learn how to spawn and control multiple robots as well as explore using a genetic algorithm to train a robot.


## System Architecture
**Prey**:
* **Wall Follower**: This code can be found in `sydney_bot.py`. The green prey robot uses the laser scanner to approach a wall, turn left, and begin following it. This is similar to the behavior of the wall follower robot in the Warmup Project. Like the previous wall follower, the prey robot continually adjusts its orientation to maintain the appropriate orientation and distance relative to the wall. The robot is also able to turn inner and outer corners, where outer corners are identified by checking the laser scanner distance ahead and to the right. When this distance is above some threshold, the robot will begin turning right. In addition, the robot uses its RGB camera to identify the orange obstacles, which is used for adjusting the robot’s direction when it is moving towards the wall.

* **Obstacle Follower**: This code can be found in `alec_bot.py`. The blue prey robot initially uses the RGB camera to search for an obstacle. If it does not observe one, it will turn in-place until it does. It will then orient itself so that it is facing the obstacle and will move forward until it is a predetermined distance away. This will be done using the robot’s laser scanner. It will then turn left and begin circling the obstacle, continuously re-orienting itself so that it is moving in a circle while maintaining a close distance to the obstacle.

* **Run Away Bot**: This code can be found in `rachel_bot.py`. The yellow prey robot uses the RGB camera feed to look for the red predator robot, similarly to how the robot in the q-learning project identified colored dumbbells. If the predator is not in view, then this prey bot spins in place. If the predator is in view, then it begins driving backwards away from the predator, using PID control (similar to but in the opposite direction as the person follower from the warmup project) to avoid walls and obstacles.

**Predator**:

* This code can be found in `kir_bot.py`. It is designed to have 8 parameters with float values ranging from [0.0, 1.0] that determine exactly how the predator behaves. The `robot_movement.py` world manager is designed to set these parameters with each new world reset based on what the genetic algorithm framework tells it to do. 3 of these parameters are weights for predator direction choice, 1 is a parameter that determines when the predator should lock on to nearby prey, and 4 are driving parameters that convert the desired predator direction into actual linear and angular velocities. The direction parameters use the scan and image data of the robot, accounting for the nearest object and the best predator in the scan in order to make a final desired angle choice that the robot should aim for.

**Genetic Algorithm**:

* This code can be found in `genetic_algorithm.py`. The code is designed to randomly create a first generation of size 100 if it does not already exist, or to load a generation from a file if we were already training. This genetic algorithm framework has a `subject` property which is the current predator configuration that needs to be trained. It has an output method called `get_params` that returns the current set of parameters which need to be tested. This method is called from `robot_movement.py` which applies these parameters to the actual predator robot. The genetic algorithm framework also has a subsequent input method called `set_score_by_capture`. This function sets the calculated fitness based on how many prey were captured out of a trial of 3 rounds. Then, it updates the `subject` as needed so that the next set of parameters can be tested. When a generation is completely tested, a new generation is immediately created that selects which parameters from the previous gen survive according to the score weights. Crossovers and mutations are then applied to this new generation, and testing begins again.

**Other**:

* **Physics Properties**: In `robot_movement.py`, Gazebo physics properties are set using a ROS service. These physics properties speed up the simulation time so that training can happen faster.

* **Enclosed World and Multiple Colored Robots**: We created the enclosed area using the Gazebo building editor. The world can be found in `enclosed_area_3.world`. We created multiple robots by expanding the given [multiple turtlebot launch file](https://github.com/intro-robotics-uchicago/multiple_turtlebot3s_in_gazebo/blob/main/launch/multiple_turtlebot3s_in_empty_world.launch) in `multi_turtlebot3.launch`. We made the robots different colors by copying the existing urdf files to our repository and combining `turtlebot3_waffle_pi.gazebo.xacro` and `turtlebot3_waffle_pi.urdf.xacro` into one file that can take a color as an argument. The `multi_turtlebot3.launch` file then uses the edited urdf file to spawn robots, giving as the argument different colors for each robot.


* **Random Spawning**: Random spawning was implemented within the `RobotMovement Class`, within the `robot_movement.py` file, largely within the reset_world function. It includes the creation of the `zero_twist` variable, which sets each robot's velocities to 0, as well as setting the orientation and position of each of the states before setting them for each robot. The placement of this code in the reset world function was because we needed to spawn the robots in a new random location each time the world reset. This spawning utilizes the Gazebo Set Model State service, which allows you to control features such as position, orientation, and velocity. While velocity would always be reset to 0 at the start of a new world, we wanted to add an element of randomness to ensure that our predator was actually “learning” a good way to catch prey generally, and not just learning for a specific situation. However, we also didn’t want problems that might arise from things like robots spawning in corners or spawning too close together, and therefore potentially giving us generations without meaningful data, so we instead decided to hardcode in the potential spawn locations, which were randomly selected from.

* **Predator Catching**: The mechanics behind catching the predators is found in the `PredatorCatch Class`, found in the file of the same name, and various elements of it are split across various functions. Within this is not just the implementation of catching prey when the predator is close to the prey, as seen in the `handle_capture_test()` function, but it tests if the predator is stuck, in `predator_stuck()`, and overall handles the question of whether or not the world needs to be reset. To test whether the predator is stuck, we keep track of the pose of the predator, specifically we keep a record of its current pose and its previous pose, and compare the robot’s position and angle to see if it is making progress. If it is stuck, as seen by it not moving, the world is reset. Similarly, if a prey is caught, as found by comparing the position of the predator and prey, the world is reset. Finally, if, after a maximum time, that can be set and is currently 30 seconds, no prey is caught, the world will be reset. Timing of the capture is also tracked, such that quicker captures are more rewarded than slower ones.

## Challenges

One challenge was that the Gazebo building editor does not allow editing after the model has been created. This resulted in accidentally creating maps that were too large or too small or too complicated, and having to restart/remake the room models. Along a similar vein, were problems when trying to handle spawning the robots, especially when it came to random spawning. We initially did not know about the services that are available in Gazebo, such as the Set Model State Service that we eventually used, and so at first we were trying to implement the random spawning within our launch file, which proved difficult, and we were unable to succeed in this until we discovered the alternative in services.

Another major challenge was getting the genetic algorithm to train the predator effectively. Our first genetic algorithm attempt was unsuccessful, with hours of training converging on a predator robot that rarely caught any prey. We had to debug the predator to ensure all parameters were working exactly as expected, we added a parameter to help the predator determine when it should focus solely on the prey rather than accounting for other obstacles, and we changed the fitness function entirely from being based on how fast the predator could capture a prey in a single run to how many successful prey captures the predator could get in 3 training runs. The latter fitness function was more effective because we were more interested in getting a predator that is consistent than a predator that is really fast but only works on occasion.


## Future Work

If we had more time, one thing we would do is increase the complexity of our world and prey behaviors. For instance, we could make the world larger and introduce additional types of obstacles; introduce multiple predator robots; or add more advanced capabilities to the prey robots, such as being able to communicate with each other and increase their speed when they identify a nearby predator.

Another feature which could be improved upon would be the random spawning, as currently there are two main elements which have slight problems. For one, while the locations are randomly selected from a list of possibilities, the orientations are not, and the robots will always start with the same orientation. Adding some randomness to the orientation would be beneficial in helping add another element to the randomness of the scenario, potentially helping improve our predators ability to learn.

The genetic algorithm we used is far from perfect, and improving it significantly would take weeks more of work. This is because it is difficult to know exactly what a perfect fitness function for generation evolution should look like. We made a simple and slightly naive fitness function which relies solely on counting successful captures out of 3 trials. We should have had more trials and larger generation size (our size was 100), along with more generations, though all of this could drag training to being hundreds of hours long, rather than just tens of hours. Our fitness function could have been improved by not only looking for perfect captures in a trial, but also looking for other useful qualities like the ability of the robot to explore the map without getting stuck or the robot getting close to capturing but not quite capturing. This sort of fitness function would be more conducive to helping traits of the predator survive that are maturing but are not yet quite good enough to capture prey, relying on the fact that they will eventually either become insignificant or will mature to being very useful in capture over lots of generations.

## Takeaways

* Communication when working in a larger group ( > 2 people ) is really important. There was one time when two members of the group accidentally began working on the predator catching with two different approaches, resulting in some duplicate work.

* We also found that specifying specific roles for each group member at the outset of the project was very helpful, as it helped each individual to plan their work schedule while ensuring an even split of the work. We were then able to make minor adjustments to the work assignments as we encountered any complications.

* We found that taking a step back and working on things such as the project proposal or the presentations helped keep us on track for the project, and acted as a way to anchor down what progress we had made up to a certain point, and to identify what reasonable adjustments to our goals may be. Effectively, having to step back from programming, and think about the project on a broader scale helped to make sure that we effectively organized the project between the four of us.

## How to Run

Launch the world and run training:

```
roslaunch final_project multi_turtlebot3.launch
roslaunch final_project action.launch
```

Generate first generation of the genetic algorithm (don't do this unless you intend to completely restart training):

```
python3 scripts/genetic_algorithm.py
```

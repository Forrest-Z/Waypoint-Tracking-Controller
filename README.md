# Waypoint-Tracking-Controller-LQR

# 1. Objective
To ensure real-time capabilities, a spline interpolator that meets continuity and smoothness restrictions has been chosen, with an optimal Linear Quadratic Regulator (LQR) lateral control that allows an intuitive and fast parameter adjustment. A velocity profiler optimizes the linear velocity as a function of the trajectory parameters. In addition, a delay compensator minimizes the effect of sensor and actuator delays in the stability of the control loop.

# 2. Controller Design
The trajectory tracking control system of an autonomous robot is composed of longitudinal and lateral controllers. The longitudinal controller is responsible for regulating the vehicle’s cruise velocity (or linear speed) while the lateral controller steers the vehicle’s wheels for path tracking. If the path is given by a set of waypoints, a trajectory interpolator is necessary to obtain smooth and optimized movements and continuous references for the actuators.

# 3. Structure 
Inputs are a set of waypoints and an external speed specification provided by the high level planner, and the estimated vehicle pose obtained from its sensors. A spline interpolator calculates a smooth trajectory, whose curvature at each point is the main input for a linear velocity profiler that adjust the longitudinal velocity of the vehicle _V_. The lateral control is an optimal LQR controller that uses the tracking errors to steer the vehicle. A delay compensation system offsets the delays introduced by the positioning system of the vehicle and the actuators. 

# References
1. Gutiérrez, R.; López-Guillén, E.; Bergasa, L.M.; Barea, R.; Pérez, Ó.; Gómez-Huélamo, C.; Arango, F.; del Egido, J.; López-Fernández, J. A Waypoint Tracking Controller for Autonomous Road Vehicles Using ROS Framework. Sensors 2020, 20, 4062. https://doi.org/10.3390/s20144062 

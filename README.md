# WaterGun

Driving code for a self-aiming water gun.

## About the Project

This project is still a work in progress, although the code is mostly complete. The next steps are to build a prototype.
The end goal is a water gun which can self-aim on both the yaw and pitch axes using stepper motors. 

The existing code in the repository uses an XBox360 Kinect in order to:
- Detect the location of people.
- Choose the person to target, based on their displacement from the current aim, direction and speed of movement, etc.
- Calculate the gun movement required to aim at that person. In particular:
  - There are constraints on the reotaitonal acceleration of both axes (due to the torque of the motors and moment of inertia of the water gun itself).
  - There is also a constraint on the maximum rotational velocity of the axes (for safety).
  - Of course we also want to be aimed in the shortest time possible.
  - For this reason, I opted to use mathematical linear programming in order to plan the acceleration of the stepper motor over a series of short timesteps.
  - In particular, the simplex method code provided by the COIN library is used.
  - I am currently using a constant-acceleration model in order to calculate the angles required to hit the target (with air resistance as a constant deceleration).
  - Whether this proves accurate enough remains to be seen. If it is particularly inaccurate, I have already created some more complex models using second order ODEs.
  - However, the code to perform such calculations becomes significantly more complex, which will in turn decrease the accuracy due to the latency of the computations.
- Convert the movement into inputs to the stepper motor. In particular, use hardware PWM for increased accuracy and to maximise the CPU time available to the aiming calculations.

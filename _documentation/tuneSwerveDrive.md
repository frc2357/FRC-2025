# How to Tune Swerve Drive PID

## Description

CTRE's velocity target is in terms of rotations per second (rps).
The only gains we need to calculate are the kS, kV, kP, and kD to create a finely tuned drive

The definition of the gains we need are:
- kS => Volts output overcome static friction (output)
- kV => output per unit of requested velocity (output/rps)
- kP => output per unit of error in velocity (output/rps)
- kD => output per unit of error derivative in velocity (output/(rps/s))

We do not use the kG, kA, and kI gains.
Why we don't use these gains:
- kG => Volts to overcome gravity => The drive is on the floor, thus there is no gravity to overcome
- kA => Used for acceleration, velocity control has no target accelerations so this gain is unused
- kI => output per unit of integrated error in velocity (output/rotation) => Used to correct an undershoot, our drive should be very aggressive and not undershoot.

## How To

There is a command called `velDrive`, this command runs the swerve along the x-axis at 2 meters per second. Bind it to a button on the `DriverControls`. This command prints the robot's velocity in meters per second to the console, and prints the motor rotations per second to the SmartDashboard.

The drive gains are located in the `TunerConstants`. The gains should be set to zero, and the closed loop output type should be `Voltage`.

The `fieldRelative` swerve request should use the `Velocity` `DriveRequestType`.

### kS
- With all gains at 0, set the kS to 1. 
- Run the `velDrive` command, the robot should be moving.
- Dial back to 0.1, if the robot continues to move, go back to 0.01
- Once you find you upper and lower bound, start cutting in the middle until the robot is not moving, and is not jingling.

### kV
- I believe in 2024, I calculated a theortical kV. It is `0.13`. 
- This number can be refined by running the `velDrive` command and watching the rotations per second of the module that is output onto the smart dashboard. The topic is "Motor rps".
- Make 0.01 incremental/decremental adjustments until the desired acceleration is acheived, and mostly stable.

### kP
- The 2024 gain was `0.05`. It is likely to be very close to this in 2025. Potentially slightly less due to the reduced robot weight. Like the other gains, use the `velDrive` command to tune.
- The desired result is for the robot to not jitter, and it is okay if it slightly overshoots, but try to eliminate it
- Tune by finding a upper and lower bound in terms of magnitude, ex(0.1, and 0.01), then start halving the difference until the desired velocity is acheived

### kD
- The 2024 gain was `0`. I would expect this to be the same for 2025. You only need this gain, if you cannot resolve an overshoot with the `kP` gain and still acheive the desired acceleration. Like the other gains, use the `velDrive` command to tune.
- To tune, follow the steps listed in kP, but start with an upper bound of 0.01 and lower bound of 0.001. It should be a very small gain.
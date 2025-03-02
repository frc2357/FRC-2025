# Best Practices for Choreo Path Generation

## Reading the Documentation

Reading [the documentation](https://choreo.autos/usage/editing-paths/) is an excellent way to get up-to-date information on the capabilities Choreo has, and subsequently, what you can do it.

## Use variables

Choreo has the ability to save essentially anything as a variable. (e.g. pose, time, velocity, acceleration) Numbers or loctations that are routinely refrenced, such as a scoring location, maximum velocity, starting location, maximum acceleration, and more, should be made a variable, and used throughout all relevant paths.

## Never use obstacles

Obstacles, under the current solver structure [cause inconsitent generation and frequent failure to converge](https://github.com/SleipnirGroup/Choreo/issues/816). The following constraints should not be used under any circumstance.

- Keep in Circle
- Keep in Rectangle
- Keep out Circle

Disable or remove all obstacles from any path you create. An alternative to obstacles is using pose or translation waypoints to force the path to go around obstacles in a predictable manner.

## Dont use whole numbers

Choreo does not like whole numbers. For example, a path may fail to converge with a max velocity constraint of 2 m/s, but can generate successfully when that same constraint is changed to 2.0001 m/s.

**Note**: This also applies to zero. If you want to apply a max angular velocity constraint to keep the robot from moving, do not use 0, use something like 0.0001 rad/s.

## Watch the path run

Once you generate the path, you should watch it run, to verify that it does what you expect it to. There are situations where it may do something unreasonable, such as overshoot on rotation between two waypoints, or make an unexpected curve. These should be watched for, and when encountered, should be fixed before you run the path.

## Curves are your friend

You may notice that Choreo will make a curve in a path if you give it 3 or more waypoints. This is normal, and should not be avoided unless necessary. Curves make a faster path with less chance of losing traction or other undesirable outcomes.

## Have max velocity and acceleration

All paths should have a max velocity, max linear acceleration, and max angular acceleration constraints from the start of the path to the end of the path. They do not need to be the same constraint throughout, and can change speeds throughout, but all paths should have velocity and acceleration constraints, to lower the chance of traction loss and to improve performance.

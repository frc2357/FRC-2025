# How to Tune Choreo Config

## MOI Calculation

**Formula**: (1/12) \* m \* ((a/12)^2 + (b/12)^2)

**Where**: m = mass of the robot in lbs a = width of the robot in inches b = length of the robot in inches

**Result**: (1/12) \* m \* ((a/12)^2 + (b/12)^2) = x \* ft^2

**Tuning**: May be necessary to tweak the MOI higher or lower. It was lower in 2024.

**Note**: The official [Choreo documentation](https://choreo.autos/) Keeps a list of ways to [estimate your MOI](https://choreo.autos/usage/estimating-moi/), consider that page as well, as this is where we originally found this formula.

Ex: In 2025 the MOI is calculated to be ~5.243 Kg/m^2, and this did not need tuning.

## Tuning the PID controllers

### Tune x and y controller

- Create a path that drives in desired direction for 3 meters
- Tune so that desired acceleration and position is met
- Will be useful to log the pose, and show on elastic
- Do not forget to set max velocity and max linear acceleration constraints, as is best practice

### Tune rotation controller

- Create path that rotates the robot 180 degress
- Tune controller until desired effect is acheived
- Do not forget to set max angular acceleration constraints, and remove linear velocity/acceleration constraints, if the path does not move

### Final Tuning

- Create a PID tuning path, path should follow a fairly sharp curve and rotate along it, 90 deg in the past
- Should span about half a field
- Tweak controllers until the robot behaves as desired
- Should have max velocity, max linear acceleration and max angular acceleration constraints similar to what will be used throughout the other autos

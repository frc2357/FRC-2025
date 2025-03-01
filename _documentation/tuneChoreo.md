# How to Tune Choreo Config

## MOI Calculation
**Formula**: (1/12) * m * ((a/12)^2 + (b/12)^2)

**Where**: m = mass of the robot in lbs a = width of the robot in inches b = length of the robot in inches

**Result**: (1/12) * m * ((a/12)^2 + (b/12)^2) = x * ft^2

**Tuning**: May be necessary to tweak the MOI higher or lower. It was lower in 2024. 

Ex: In 2024 the calculated MOI was ~201 but tuned MOI was 130

## Tuning the PID controllers

### Tune x and y controller
- Create a path that drives in desired direction for 3 meters
- Tune so that desired acceleration and position is met
- Will be useful to log the pose, and show on smartdashboard

### Tune rotation controller
- Create path that rotates the robot 180 degress
- Tune controller until desired effect is acheived

### Final Tuning
- Create a PID tuning path, path should follow a fairly sharp curve and rotate along it
- Should span about half a field
- Tweak controllers until the robot behaves as desired
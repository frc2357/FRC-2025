package frc.robot.util;

import static frc.robot.Constants.COLLISION_DETECTION.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.Units;

public class CollisionDetection {

  public static boolean willHitReef(
    Pose2d currPose,
    Pose2d targetPose,
    double... interpolationPercentages
  ) {
    Transform2d currToTargetTransform = new Transform2d(currPose, targetPose);
    for (double percentage : interpolationPercentages) {
      Transform2d transformToUse = currToTargetTransform.times(percentage);
      Pose2d interpolatedPose = currPose.transformBy(transformToUse);
      // if true, collision with reef is likely
      if (
        SATCollisionDetector.hasCollided(
          SATCollisionDetector.makePolyFromRobotPose(interpolatedPose),
          REEF_SAT_POLY,
          REEF_BOUNDARY.in(Units.Meters)
        )
      ) {
        return true;
      }
    }
    return false;
  }
}

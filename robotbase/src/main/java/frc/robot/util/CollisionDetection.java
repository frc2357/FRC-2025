package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.COLLISION_DETECTION.*;
import static frc.robot.Constants.FIELD.REEF.*;
import static frc.robot.Constants.FIELD_CONSTANTS.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.Units;
import frc.robot.Constants.FIELD_CONSTANTS;
import frc.robot.util.SATCollisionDetector.SATVector;

public class CollisionDetection {

  /**
   * Exists purely to keep a ton of lines out of the {@link frc.robot.Constants.COLLISION_DETECTION Constants} file.
   * @return A polygon of the reef, in SATVectors, with the collision tolerance applied. Larger than the actual reef.
   */
  public static SATVector[] createReefPolygon() {
    return new SATVector[] {
      new SATVector(
        BOTTOM_LEFT_CORNER.transformBy(
          new Transform2d(CENTER, BOTTOM_LEFT_CORNER).times(
            (1 /
              Math.abs(
                Utility.findDistanceBetweenPoses(CENTER, BOTTOM_LEFT_CORNER)
              )) *
            COLLISION_TOLERANCE.in(Meters)
          )
        )
      ),
      new SATVector(
        TOP_LEFT_CORNER.transformBy(
          new Transform2d(CENTER, TOP_LEFT_CORNER).times(
            (1 /
              Math.abs(
                Utility.findDistanceBetweenPoses(CENTER, TOP_LEFT_CORNER)
              )) *
            COLLISION_TOLERANCE.in(Meters)
          )
        )
      ),
      new SATVector(
        TOP_CORNER.transformBy(
          new Transform2d(CENTER, TOP_CORNER).times(
            (1 /
              Math.abs(Utility.findDistanceBetweenPoses(CENTER, TOP_CORNER))) *
            COLLISION_TOLERANCE.in(Meters)
          )
        )
      ),
      new SATVector(
        TOP_RIGHT_CORNER.transformBy(
          new Transform2d(CENTER, TOP_RIGHT_CORNER).times(
            (1 /
              Math.abs(
                Utility.findDistanceBetweenPoses(CENTER, TOP_RIGHT_CORNER)
              )) *
            COLLISION_TOLERANCE.in(Meters)
          )
        )
      ),
      new SATVector(
        BOTTOM_RIGHT_CORNER.transformBy(
          new Transform2d(CENTER, BOTTOM_RIGHT_CORNER).times(
            (1 /
              Math.abs(
                Utility.findDistanceBetweenPoses(CENTER, BOTTOM_RIGHT_CORNER)
              )) *
            COLLISION_TOLERANCE.in(Meters)
          )
        )
      ),
      new SATVector(
        BOTTOM_CORNER.transformBy(
          new Transform2d(CENTER, BOTTOM_CORNER).times(
            (1 /
              Math.abs(
                Utility.findDistanceBetweenPoses(CENTER, BOTTOM_CORNER)
              )) *
            COLLISION_TOLERANCE.in(Meters)
          )
        )
      ),
    };
  }

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

  public static boolean isPoseInField(Pose3d pose) {
    return isPoseInField(pose.toPose2d());
  }

  public static boolean isPoseInField(Pose2d pose) {
    return !(
      pose.getX() < -FIELD_BORDER_MARGIN.in(Meters) ||
      pose.getX() >
      FIELD_CONSTANTS.FIELD_LENGTH.plus(FIELD_BORDER_MARGIN).in(Meters) ||
      pose.getY() < -FIELD_BORDER_MARGIN.in(Meters) ||
      pose.getY() >
      FIELD_CONSTANTS.FIELD_WIDTH.plus(FIELD_BORDER_MARGIN).in(Meters)
    );
  }

  public static boolean isOnBlueSide(Pose2d pose) {
    return !(
      pose.getX() < -FIELD_BORDER_MARGIN.in(Meters) ||
      pose.getX() >
      FIELD_CONSTANTS.FIELD_LENGTH.div(2)
        .plus(FIELD_BORDER_MARGIN)
        .in(Meters) ||
      pose.getY() < -FIELD_BORDER_MARGIN.in(Meters) ||
      pose.getY() >
      FIELD_CONSTANTS.FIELD_WIDTH.plus(FIELD_BORDER_MARGIN).in(Meters)
    );
  }

  public static boolean isOnRedSide(Pose2d pose) {
    return !(
      pose.getX() < -FIELD_BORDER_MARGIN.in(Meters) ||
      pose.getX() <
      FIELD_CONSTANTS.FIELD_LENGTH.div(2)
        .plus(FIELD_BORDER_MARGIN)
        .in(Meters) ||
      pose.getY() < -FIELD_BORDER_MARGIN.in(Meters) ||
      pose.getY() >
      FIELD_CONSTANTS.FIELD_WIDTH.plus(FIELD_BORDER_MARGIN).in(Meters)
    );
  }
}

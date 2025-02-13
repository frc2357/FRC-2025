package frc.robot;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.DRIVE_TO_POSE;
import frc.robot.Constants.FIELD.REEF;
import frc.robot.commands.drive.DriveToPoseHandler;
import frc.robot.util.Utility;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class DriveToPoseHandlerTests extends DriveToPoseHandler {

  static final double DELTA = 1E-2;

  @Override
  public void initialize() {}

  @Override
  public void execute() {}

  @BeforeEach
  void setup() {}

  @Test
  void isNOTAtTarTest() {
    Pose2d currPose = Pose2d.kZero;
    Pose2d currTar = new Pose2d(4, 4, Rotation2d.kZero);
    boolean result = super.isAtTarget(currTar, currPose);
    assertEquals(false, result);
  }

  @Test
  void isAtTarTest() {
    Pose2d currPose = new Pose2d(1.00001, 1.0001, Rotation2d.kZero);
    Pose2d currTar = new Pose2d(1, 1, Rotation2d.kZero);
    boolean result = super.isAtTarget(currTar, currPose);
    assertEquals(true, result);
  }

  @Test
  void findNewTargetFinalApproachTest() {
    Pose2d finalGoal = REEF.BRANCH_A;
    Pose2d currPose = finalGoal.plus(new Transform2d(-.5, 0, Rotation2d.kZero));
    Pose2d currTar = REEF.BRANCH_A.plus(
      new Transform2d(-0.05, 0, Rotation2d.kZero)
    );
    m_finalGoal = finalGoal;
    Pose2d result = super.getNewTarget(currTar, currPose);
    assertEquals(finalGoal, result);
  }

  @Test
  void InterpolateTest() {
    Pose2d poseToInterp = Pose2d.kZero;
    Pose2d goal = new Pose2d(1, 0, Rotation2d.kZero);
    Pose2d result = super.interpolateTarget(poseToInterp, goal);
    Distance distInterpolated = Units.Meters.of(
      Utility.findDistanceBetweenPoses(poseToInterp, result)
    );
    assertEquals(
      distInterpolated.in(Meters),
      DRIVE_TO_POSE.INTERPOLATION_DISTANCE.in(Meters),
      0.01
    );
  }
}

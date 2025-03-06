package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;

public class TuningPathFinal extends AutoBase {

  public TuningPathFinal() {
    super("Tuning Path Final", "TuningPathFinal");
    Pose2d startingPose = m_startTraj
      .getInitialPose()
      .orElse(new Pose2d(-1, -1, new Rotation2d(-1)));
    Pose2d finalPose = m_startTraj
      .getFinalPose()
      .orElse(new Pose2d(-1, -1, new Rotation2d(-1)));
    m_startTraj
      .active()
      .onTrue(
        new InstantCommand(() ->
          System.out.println("STARTING POSE - CHOREO: " + startingPose)
        )
      );
    m_startTraj
      .atTime(1)
      .onTrue(
        new InstantCommand(() ->
          System.out.println(
            "POSE - CHOREO: " + Robot.swerve.getFieldRelativePose2d()
          )
        )
      );
    m_startTraj
      .atTime(2)
      .onTrue(
        new InstantCommand(() ->
          System.out.println(
            "POSE - CHOREO: " + Robot.swerve.getFieldRelativePose2d()
          )
        )
      );
    m_startTraj
      .done()
      .onTrue(
        new InstantCommand(() ->
          System.out.println(
            "FINAL POSE - CHOREO: " +
            Robot.swerve.getFieldRelativePose2d() +
            "\nDIFF BETWEEN DESIRED AND ACTUAL: " +
            new Transform2d(finalPose, Robot.swerve.getFieldRelativePose2d())
          )
        )
      );
  }
}

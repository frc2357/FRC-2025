package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.PhotonVisionCamera;

public class TuningPathY extends AutoBase {

  public TuningPathY() {
    super("Tuning Path Y", "TuningPathY");
    Pose2d startingPose = m_startTraj.getInitialPose().orElse(Pose2d.kZero);
    Pose2d finalPose = m_startTraj.getFinalPose().orElse(Pose2d.kZero);
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

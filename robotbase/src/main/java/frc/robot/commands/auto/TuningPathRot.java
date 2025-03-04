package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.PhotonVisionCamera;

public class TuningPathRot extends AutoBase {

  public TuningPathRot() {
    super("Tuning Path Rot", "TuningPathRotation");
    Pose2d startingPose = m_startTraj.getInitialPose().get();
    Pose2d finalPose = m_startTraj.getFinalPose().get();
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
            "\nFINAL POSE - PHOTON: " +
            PhotonVisionCamera.getLastEstimatedPose().estimatedPose.toPose2d() +
            "\nDIFF BETWEEN DESIRED AND ACTUAL: " +
            new Transform2d(finalPose, Robot.swerve.getFieldRelativePose2d())
          )
        )
      );
  }
}

package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import org.photonvision.EstimatedRobotPose;

public record PipelineResult(Pose2d robotPose, EstimatedRobotPose rawResult) {
  public PipelineResult(EstimatedRobotPose estimate) {
    this(estimate.estimatedPose.toPose2d(), estimate);
  }
}

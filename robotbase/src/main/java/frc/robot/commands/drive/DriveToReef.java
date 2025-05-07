package frc.robot.commands.drive;

import static frc.robot.Constants.FIELD.REEF.BRANCHES;
import static frc.robot.Constants.FIELD.REEF.BRANCH_A;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DRIVE_TO_POSE.BRANCH_GOAL;
import frc.robot.Constants.FIELD.REEF;
import frc.robot.Robot;

public class DriveToReef extends DriveToPoseHandler {

  private Pose2d m_goal;

  public DriveToReef(RouteAroundReef routeAroundReef, Pose2d goal) {
    this(routeAroundReef, BRANCH_A, null);
  }

  public DriveToReef(
    RouteAroundReef routeAroundReef,
    Pose2d goal,
    Command finalApproachCommand
  ) {
    super(routeAroundReef, finalApproachCommand);
    m_goal = goal;
    m_finalGoal = goal;
  }

  @Override
  public void initialize() {
    m_finalGoal = m_goal;
    super.initialize();
  }
}

package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveToReef extends DriveToPoseHandler {

  private Pose2d m_goal;

  public DriveToReef(RouteAroundReef routeAroundReef, Pose2d goal) {
    this(routeAroundReef, goal, null);
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

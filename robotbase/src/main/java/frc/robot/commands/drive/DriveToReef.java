package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;

public class DriveToReef extends DriveToPoseHandler {

  public DriveToReef(RouteAroundReef routeAroundReef) {
    this(routeAroundReef, null);
  }

  public DriveToReef(
    RouteAroundReef routeAroundReef,
    Command finalApproachCommand
  ) {
    super(routeAroundReef, finalApproachCommand);
  }
}

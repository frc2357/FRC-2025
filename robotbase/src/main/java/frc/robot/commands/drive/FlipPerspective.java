package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class FlipPerspective extends Command {

  public FlipPerspective() {
    addRequirements(Robot.swerve);
  }

  @Override
  public void initialize() {
    double currentDegrees = Robot.swerve
      .getOperatorForwardDirection()
      .getDegrees();

    if (currentDegrees == 0) {
      Robot.swerve.setOperatorPerspectiveForward(new Rotation2d(Math.PI));
    } else {
      Robot.swerve.setOperatorPerspectiveForward(new Rotation2d(0));
    }
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}

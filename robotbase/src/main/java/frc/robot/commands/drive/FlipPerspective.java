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
    Robot.swerve.setOperatorPerspectiveForward(
      Robot.swerve.getOperatorForwardDirection().unaryMinus()
    );
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}

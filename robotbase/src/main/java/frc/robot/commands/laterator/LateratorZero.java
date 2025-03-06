package frc.robot.commands.laterator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LATERATOR;
import frc.robot.Robot;

public class LateratorZero extends Command {

  public LateratorZero() {
    addRequirements(Robot.laterator);
  }

  @Override
  public void initialize() {
    Robot.laterator.setSpeed(LATERATOR.RETURN_SPEED);
  }

  @Override
  public boolean isFinished() {
    return Robot.laterator.isAtZero();
  }

  @Override
  public void end(boolean interrupted) {
    Robot.laterator.stop();
    if (!interrupted) {
      Robot.laterator.setZero();
    }
  }
}

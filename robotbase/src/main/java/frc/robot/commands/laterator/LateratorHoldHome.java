package frc.robot.commands.laterator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;

public class LateratorHoldHome extends Command {

  public LateratorHoldHome() {
    addRequirements(Robot.laterator);
  }

  @Override
  public void initialize() {
    if (Robot.laterator.startedAtZero()) {
      Robot.laterator.setTargetDistance(Constants.ELEVATOR.SETPOINTS.HOME);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.laterator.stop();
  }
}

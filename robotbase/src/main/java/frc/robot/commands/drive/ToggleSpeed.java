package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ToggleSpeed extends Command {

  public void initialize() {
    Robot.swerve.togglespeed = true;
  }

  public void end(boolean interrupted) {
    Robot.swerve.togglespeed = false;
  }
}

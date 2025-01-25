package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class InitSwerve extends Command {

  @Override
  public void initialize() {
    Robot.swerve.setOperatorPerspectiveForward(Robot.alliance);
    Robot.swerve.seedFieldCentric();
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}

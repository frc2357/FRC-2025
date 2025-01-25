package frc.robot.commands.util;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.InitSwerve;

public class InitRobotCommand extends SequentialCommandGroup {

  public InitRobotCommand() {
    super(new SetAlliance(), new InitSwerve());
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}

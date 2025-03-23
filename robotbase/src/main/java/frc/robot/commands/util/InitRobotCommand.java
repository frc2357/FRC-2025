package frc.robot.commands.util;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.InitSwerve;
import frc.robot.commands.laterator.SetStartedAtZero;

public class InitRobotCommand extends SequentialCommandGroup {

  public InitRobotCommand() {
    super(new SetAlliance(), new InitSwerve(), new SetStartedAtZero());
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}

package frc.robot.commands.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class SetAlliance extends Command {

  private Alliance m_fetchedAlliance;

  public SetAlliance() {
    m_fetchedAlliance = null;
  }

  @Override
  public void execute() {
    m_fetchedAlliance = DriverStation.getAlliance().orElse(null);
  }

  @Override
  public boolean isFinished() {
    return m_fetchedAlliance != null;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.alliance = m_fetchedAlliance;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}

package frc.robot.commands.state;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import java.util.Optional;

public class GetAlliance extends Command {
  private Optional<Alliance> m_alliance;

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_alliance = DriverStation.getAlliance();
  }

  @Override
  public boolean isFinished() {
    return !m_alliance.isEmpty();
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      DriverStation.reportError(
          "[GetAlliance] Interrupted while trying to retrieve Alliance", false);
      return;
    }

    Robot.state.setAlliance(m_alliance.get());
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}

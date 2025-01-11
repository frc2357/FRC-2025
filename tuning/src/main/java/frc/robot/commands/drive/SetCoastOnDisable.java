package frc.robot.commands.drive;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SWERVE;
import frc.robot.Robot;

public class SetCoastOnDisable extends Command {
  Timer m_timer = new Timer();

  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
  }

  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(SWERVE.TIME_TO_COAST.baseUnitMagnitude());
  }

  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      Robot.swerve.configNeutralMode(NeutralModeValue.Coast);
    }
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}

package frc.robot.commands.util;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import java.util.function.DoubleSupplier;

public class VariableWaitCommand extends WaitCommand {
  DoubleSupplier m_getSeconds;

  public VariableWaitCommand(DoubleSupplier getSeconds) {
    super(0);
    m_getSeconds = getSeconds;
  }

  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_getSeconds.getAsDouble());
  }
}

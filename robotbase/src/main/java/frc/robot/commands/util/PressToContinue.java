package frc.robot.commands.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class PressToContinue extends Command {
  private Trigger m_button;
  private boolean m_wasReleased;

  public PressToContinue(Trigger button) {
    m_button = button;
  }

  @Override
  public void initialize() {
    m_wasReleased = false;
  }

  @Override
  public boolean isFinished() {
    if (m_button.getAsBoolean()) {
      if (m_wasReleased) {
        return true;
      }
    } else {
      m_wasReleased = true;
    }
    return false;
  }
}

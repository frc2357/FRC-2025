package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.SelectCommand;
import java.util.Map;

public class AlgaeChooser {

  private boolean m_currentPosition = false;

  private SelectCommand<Boolean> m_command = new SelectCommand<Boolean>(
    Map.ofEntries(
      Map.entry(true, new AlgaeIntake()),
      Map.entry(false, new AlgaeScore())
    ),
    this::supplier
  );

  public SelectCommand<Boolean> getSelectCommand() {
    return m_command;
  }

  private Boolean supplier() {
    m_currentPosition = !m_currentPosition;
    return m_currentPosition;
  }
}

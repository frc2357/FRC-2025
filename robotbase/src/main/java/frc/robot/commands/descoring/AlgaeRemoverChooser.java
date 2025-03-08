package frc.robot.commands.descoring;

import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.commands.scoring.coral.CoralHome;
import java.util.Map;

public class AlgaeRemoverChooser {

  private enum POSITION {
    HOME,
    LOW,
    HIGH,
  }

  private POSITION[] m_positions = new POSITION[] {
    POSITION.HOME,
    POSITION.LOW,
    POSITION.HIGH,
  };

  private int m_positionLength = m_positions.length;

  private int m_currentPosition = 0;

  private SelectCommand<POSITION> m_command = new SelectCommand<POSITION>(
    Map.ofEntries(
      Map.entry(POSITION.HOME, new CoralHome()),
      Map.entry(POSITION.LOW, new RemoveAlgaeLow()),
      Map.entry(POSITION.HIGH, new RemoveAlgaeHigh())
    ),
    this::levelSupplier
  );

  public SelectCommand<POSITION> getSelectCommand() {
    return m_command;
  }

  private POSITION levelSupplier() {
    m_currentPosition = m_currentPosition == m_positionLength - 1
      ? 0
      : m_currentPosition + 1;
    return m_positions[m_currentPosition];
  }
}

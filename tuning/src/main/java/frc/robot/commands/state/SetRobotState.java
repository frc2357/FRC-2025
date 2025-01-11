package frc.robot.commands.state;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.state.RobotState;

public class SetRobotState extends Command {
  private RobotState.State m_state;

  public SetRobotState(RobotState.State state) {
    m_state = state;
  }

  @Override
  public void initialize() {
    Robot.state.setState(m_state);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}

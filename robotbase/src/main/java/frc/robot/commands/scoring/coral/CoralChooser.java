package frc.robot.commands.scoring.coral;

import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.Robot;
import frc.robot.commands.intake.CoralIntake;
import java.util.Map;

public class CoralChooser {

  private SelectCommand<Boolean> m_command = new SelectCommand<Boolean>(
    Map.ofEntries(
      Map.entry(true, new CoralScore()),
      Map.entry(false, new CoralIntake())
    ),
    this::supplier
  );

  public SelectCommand<Boolean> getSelectCommand() {
    return m_command;
  }

  private boolean supplier() {
    return Robot.coralRunner.isOuttakeBeamBroken();
  }
}

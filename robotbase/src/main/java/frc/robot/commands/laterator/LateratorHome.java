package frc.robot.commands.laterator;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;

public class LateratorHome extends SequentialCommandGroup {

  public LateratorHome() {
    super(
      new LateratorSetDistance(() -> Constants.LATERATOR.SETPOINTS.HOME),
      new LateratorZero()
    );
  }
}

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.rumble.RumbleDriverController;
import frc.robot.commands.scoring.CoralHome;

public class CoralRetract extends SequentialCommandGroup {

  public CoralRetract() {
    super(
      new CoralHome().alongWith(new CoralSettle()),
      new InstantCommand(() -> {
        if (
          Robot.coralRunner.isOuttakeBeamBroken() && !RobotState.isAutonomous()
        ) {
          new RumbleDriverController().schedule();
        }
      })
    );
  }
}

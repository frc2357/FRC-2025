package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.rumble.RumbleDriverController;
import frc.robot.commands.scoring.CoralHome;

public class CoralRetract extends ParallelCommandGroup {

  public CoralRetract() {
    this(true);
  }

  public CoralRetract(boolean zero) {
    super(
      new CoralHome(() -> zero).alongWith(new CoralSettle()),
      new ConditionalCommand(
        new RumbleDriverController(),
        new InstantCommand(),
        () -> {
          return (
            Robot.coralRunner.isOuttakeBeamBroken() &&
            !RobotState.isAutonomous()
          );
        }
      )
    );
  }
}

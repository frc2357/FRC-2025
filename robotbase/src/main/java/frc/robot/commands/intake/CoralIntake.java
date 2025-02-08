package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.coralRunner.CoralRunnerSetVelocity;
import frc.robot.commands.elevator.ElevatorSetDistance;
import frc.robot.commands.laterator.LateratorSetDistance;

public class CoralIntake extends SequentialCommandGroup {

  public CoralIntake() {
    super(
      new ParallelCommandGroup(
        new LateratorSetDistance(Constants.LATERATOR.SETPOINTS.INTAKE),
        new ElevatorSetDistance(Constants.ELEVATOR.SETPOINTS.INTAKE),
        new CoralRunnerSetVelocity(
          Constants.CORAL_RUNNER.FAST_INTAKE_VELOCITY
        ).until(Robot.coralRunner::isIntakeBeamBroken)
      ),
      new ParallelCommandGroup(
        new LateratorSetDistance(Constants.LATERATOR.SETPOINTS.HOME),
        new ElevatorSetDistance(Constants.ELEVATOR.SETPOINTS.HOME),
        new CoralRunnerSetVelocity(
          Constants.CORAL_RUNNER.SLOW_INTAKE_VELOCITY
        ).until(Robot.coralRunner::isOuttakeBeamBroken)
      )
    );
  }
}

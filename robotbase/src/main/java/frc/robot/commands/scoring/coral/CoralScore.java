package frc.robot.commands.scoring.coral;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.coralRunner.CoralRunnerSetSpeed;
import frc.robot.commands.laterator.LateratorSetDistance;
import frc.robot.util.Utility;
import frc.robot.util.Utility;

public class CoralScore extends SequentialCommandGroup {

  public CoralScore() {
  public CoralScore() {
    super(
      new LateratorSetDistance(() -> {
        if (
          Utility.isWithinTolerance(
            Robot.elevator.getDistance().in(Units.Inches),
            Constants.ELEVATOR.SETPOINTS.L4_PREPOSE.in(Units.Inches),
            Constants.ELEVATOR.L4_DETECTION_TOLERANCE.in(Units.Inches)
          )
        ) {
          return Constants.LATERATOR.SETPOINTS.L4_PREPOSE;
        }
      }),
      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          new WaitUntilCommand(
            () ->
              !Robot.coralRunner.isOuttakeBeamBroken() &&
              !Robot.coralRunner.isIntakeBeamBroken()
          ),
          new WaitCommand(Constants.CORAL_RUNNER.SCORING_WAIT_TIME)
        ),
        new ConditionalCommand(
          new CoralRunnerSetSpeed(Constants.CORAL_RUNNER.SCORING_PERCENT_L4),
          new CoralRunnerSetSpeed(Constants.CORAL_RUNNER.SCORING_PERCENT_OTHER),
          () ->
            Utility.isWithinTolerance(
              Robot.elevator.getDistance().in(Units.Inches),
              Constants.ELEVATOR.SETPOINTS.L4_PREPOSE.in(Units.Inches),
              Constants.ELEVATOR.L4_DETECTION_TOLERANCE.in(Units.Inches)
            )
        )
      ),
      new CoralHome()
        new ConditionalCommand(
          new CoralRunnerSetSpeed(Constants.CORAL_RUNNER.SCORING_PERCENT_L4),
          new CoralRunnerSetSpeed(Constants.CORAL_RUNNER.SCORING_PERCENT_OTHER),
          () ->
            Utility.isWithinTolerance(
              Robot.elevator.getDistance().in(Units.Inches),
              Constants.ELEVATOR.SETPOINTS.L4_PREPOSE.in(Units.Inches),
              Constants.ELEVATOR.L4_DETECTION_TOLERANCE.in(Units.Inches)
            )
        )
      ),
      new CoralHome()
    );
  }
}

package frc.robot.commands.scoring.coral;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.CORAL_RUNNER;
import frc.robot.Robot;
import frc.robot.commands.coralRunner.CoralRunnerSetSpeed;
import frc.robot.commands.laterator.LateratorSetDistance;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class CoralScore extends SequentialCommandGroup {

  public CoralScore(Supplier<Distance> lateratorDistance) {
    this(lateratorDistance, () -> true);
  }

  public CoralScore(
    Supplier<Distance> lateratorDistance,
    BooleanSupplier zero
  ) {
    this(lateratorDistance, () -> true, () -> CORAL_RUNNER.SCORING_PERCENT);
  }

  public CoralScore(
    Supplier<Distance> lateratorDistance,
    BooleanSupplier zero,
    Supplier<Dimensionless> runnerSpeed
  ) {
    super(
      new LateratorSetDistance(lateratorDistance).withDeadline(
        new WaitCommand(.7)
      ),
      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          new WaitUntilCommand(Robot.coralRunner::isOuttakeBeamBroken),
          new WaitCommand(Constants.CORAL_RUNNER.SCORING_WAIT_TIME)
        ),
        new CoralRunnerSetSpeed(runnerSpeed.get())
      ),
      new CoralHome(zero)
    );
  }
}

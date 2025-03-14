package frc.robot.commands.scoring.coral;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.Robot;
import frc.robot.commands.intake.CoralIntake;

public class CoralIntakeScoreConditional extends ConditionalCommand {

  public CoralIntakeScoreConditional() {
    super(
      new CoralScore(),
      new CoralIntake(),
      () ->
        Robot.coralRunner.isOuttakeBeamBroken() &&
        Robot.coralRunner.isOuttakeBeamBroken()
    );
  }
}

package frc.robot.commands.laterator;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;

public class LateratorFullZero extends SequentialCommandGroup {

  public LateratorFullZero() {
    super(
      new LateratorZero(),
      new ConditionalCommand(
        new InstantCommand(),
        new LateratorZeroStall(),
        Robot.laterator::isAtZero
      )
    );
  }
}

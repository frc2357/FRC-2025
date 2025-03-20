package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ELEVATOR;
import frc.robot.Constants.LATERATOR;
import frc.robot.commands.elevator.ElevatorHome;
import frc.robot.commands.elevator.ElevatorSetDistance;
import frc.robot.commands.laterator.LateratorHome;
import frc.robot.commands.laterator.LateratorSetDistance;
import java.util.function.BooleanSupplier;

public class CoralHome extends ConditionalCommand {

  public CoralHome() {
    this(() -> true);
  }

  public CoralHome(BooleanSupplier zero) {
    super(
      new LateratorHome()
        .alongWith(new WaitCommand(0.2).andThen(new ElevatorHome())),
      new LateratorSetDistance(LATERATOR.SETPOINTS.HOME)
        .alongWith(
          new WaitCommand(0.2).andThen(
            new ElevatorSetDistance(ELEVATOR.SETPOINTS.HOME)
          )
        )
        .withDeadline(new WaitCommand(1.5)),
      zero
    );
  }
}

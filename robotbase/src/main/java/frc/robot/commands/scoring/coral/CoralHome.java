package frc.robot.commands.scoring.coral;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ELEVATOR;
import frc.robot.Constants.LATERATOR;
import frc.robot.commands.elevator.ElevatorHome;
import frc.robot.commands.elevator.ElevatorSetDistance;
import frc.robot.commands.laterator.LateratorHome;
import frc.robot.commands.laterator.LateratorSetDistance;
import java.util.function.BooleanSupplier;

public class CoralHome extends ParallelCommandGroup {

  public CoralHome() {
    this(() -> true);
  }

  public CoralHome(BooleanSupplier zero) {
    super(
      new ConditionalCommand(
        new LateratorHome().alongWith(new ElevatorHome()),
        new LateratorSetDistance(() -> LATERATOR.SETPOINTS.HOME)
          .alongWith(new ElevatorSetDistance(ELEVATOR.SETPOINTS.HOME))
          .withDeadline(new WaitCommand(1.5)),
        zero
      )
    );
  }
}

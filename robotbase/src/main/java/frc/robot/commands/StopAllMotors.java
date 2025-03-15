package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.coralRunner.CoralRunnerStop;
import frc.robot.commands.elevator.ElevatorStop;
import frc.robot.commands.laterator.LateratorStop;

public class StopAllMotors extends ParallelCommandGroup {

  public StopAllMotors() {
    super(new CoralRunnerStop(), new LateratorStop(), new ElevatorStop());
  }
}

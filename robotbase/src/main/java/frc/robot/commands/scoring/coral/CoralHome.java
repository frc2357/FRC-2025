package frc.robot.commands.scoring.coral;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.elevator.ElevatorHome;
import frc.robot.commands.elevator.ElevatorZero;
import frc.robot.commands.laterator.LateratorHome;

public class CoralHome extends ParallelCommandGroup {

  public CoralHome() {
    super(new LateratorHome(), new ElevatorHome());
  }
}

package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.elevator.ElevatorAmpLimitZero;
import frc.robot.commands.laterator.LateratorFullZero;

public class CoralZero extends ParallelCommandGroup {

  public CoralZero() {
    super(new LateratorFullZero(), new ElevatorAmpLimitZero());
  }
}

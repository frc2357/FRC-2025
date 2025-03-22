package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.elevator.ElevatorHallEffectZero;
import frc.robot.commands.laterator.LateratorZero;

public class CoralZero extends ParallelCommandGroup {

  public CoralZero() {
    super(new LateratorZero(), new ElevatorHallEffectZero());
  }
}

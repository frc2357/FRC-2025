package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.elevator.ElevatorHome;
import frc.robot.commands.laterator.LateratorHome;
import frc.robot.commands.laterator.LateratorToSafeSpot;

public class CoralHome extends SequentialCommandGroup {

  public CoralHome() {
    super(
      new LateratorToSafeSpot(),
      new ParallelCommandGroup(new LateratorHome(), new ElevatorHome())
    );
  }
}

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.scoring.CoralHome;

public class CoralRetract extends ParallelCommandGroup {

  public CoralRetract() {
    this(true);
  }

  public CoralRetract(boolean zero) {
    super(new CoralHome(() -> zero).alongWith(new CoralSettle()));
  }
}

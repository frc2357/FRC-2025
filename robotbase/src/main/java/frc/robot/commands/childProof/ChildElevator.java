package frc.robot.commands.childProof;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.commands.elevator.ElevatorHome;
import frc.robot.commands.elevator.ElevatorSetDistance;
import frc.robot.commands.util.PressToContinue;

public class ChildElevator extends SequentialCommandGroup {
    public ChildElevator(Trigger button) {
        super(
                new ElevatorSetDistance(
                        Constants.ELEVATOR.SETPOINTS.L2_PREPOSE),
                new PressToContinue(button), new ElevatorHome());

    }

}

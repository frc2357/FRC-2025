package frc.robot.commands.algaeRunner;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class AlgaeRunnerStop extends Command {
    public AlgaeRunnerStop() {
        addRequirements(Robot.algaeRunner);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        Robot.algaeRunner.stop();
    }
}

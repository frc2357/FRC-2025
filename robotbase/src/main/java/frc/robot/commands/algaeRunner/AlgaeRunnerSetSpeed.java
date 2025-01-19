package frc.robot.commands.algaeRunner;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class AlgaeRunnerSetSpeed extends Command {
    private double m_speed;

    public AlgaeRunnerSetSpeed(double speed) {
        m_speed = speed;
        addRequirements(Robot.algaeRunner);
    }

    @Override
    public void initialize() {
        Robot.algaeRunner.setAxisSpeed(m_speed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Robot.algaeRunner.stop();
    }
}

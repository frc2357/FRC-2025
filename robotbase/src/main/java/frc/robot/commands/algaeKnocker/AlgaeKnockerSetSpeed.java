package frc.robot.commands.algaeKnocker;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class AlgaeKnockerSetSpeed extends Command {
    private double m_speed;

    public AlgaeKnockerSetSpeed(double speed) {
        addRequirements(Robot.algaeKnocker);
        m_speed = speed;
    }

    @Override
    public void initialize() {
        Robot.algaeRunner.setSpeed(m_speed);
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

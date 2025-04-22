package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;

public class CoralSettle extends Command {

  private Timer m_timer;
  private boolean m_backingOut;

  private boolean isStalling() {
    return Robot.coralRunner.isStalling() && m_timer.hasElapsed(0.1);
  }

  public CoralSettle() {
    m_timer = new Timer();
    addRequirements(Robot.coralRunner);
  }

  @Override
  public void initialize() {
    m_timer.start();
  }

  @Override
  public void execute() {
    if (
      m_backingOut &&
      !m_timer.hasElapsed(Constants.CORAL_RUNNER.BACKOUT_TIME_SECONDS)
    ) {
      return;
    }

    if (isStalling()) {
      Robot.coralRunner.setSpeed(Constants.CORAL_RUNNER.BACK_OUT_PERCENT);
      m_timer.reset();
      m_backingOut = true;
    } else {
      Robot.coralRunner.setSpeed(Constants.CORAL_RUNNER.SLOW_INTAKE_PERCENT);
      m_backingOut = false;
    }
  }

  @Override
  public boolean isFinished() {
    return Robot.coralRunner.isOuttakeBeamBroken();
  }

  @Override
  public void end(boolean interrupted) {
    Robot.coralRunner.stop();
    m_timer.stop();
    m_timer.reset();
  }
}

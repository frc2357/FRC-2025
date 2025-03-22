package frc.robot.commands.rumble;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CONTROLLER;
import frc.robot.Robot;

public class RumbleCodriverController extends Command {

  Timer timer = new Timer();

  @Override
  public void initialize() {
    Robot.codriverControls.setRumble(CONTROLLER.CODRIVE_RUMBLE_INTENSITY);
    timer.reset();
    timer.start();
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(CONTROLLER.CODRIVE_RUMBLE_SECONDS);
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
    Robot.codriverControls.setRumble(0);
  }
}

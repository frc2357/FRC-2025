package frc.robot.commands.rumble;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CONTROLLER;
import frc.robot.Robot;

public class ClearButtonboard extends Command {

  Timer timer = new Timer();

  @Override
  public void initialize() {
    Robot.buttonboard.setRumble(CONTROLLER.BUTTONBOARD_RUMBLE_INTENSITY);
    timer.reset();
    timer.start();
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(CONTROLLER.BUTTONBOARD_RUMBLE_SECONDS);
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
    Robot.buttonboard.setRumble(0);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}

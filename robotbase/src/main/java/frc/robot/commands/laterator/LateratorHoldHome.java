package frc.robot.commands.laterator;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;

public class LateratorHoldHome extends Command {

  private static final Alert m_holdingHomeAlert = new Alert(
    "Laterator Alerts",
    "Hold Home",
    AlertType.kInfo
  );

  public LateratorHoldHome() {
    addRequirements(Robot.laterator);
    m_holdingHomeAlert.setText("Not Yet Run");
  }

  @Override
  public void initialize() {
    if (Robot.laterator.startedAtZero()) {
      m_holdingHomeAlert.setText("Holding Home");
      Robot.laterator.setTargetDistance(Constants.LATERATOR.SETPOINTS.HOME);
    } else {
      m_holdingHomeAlert.setText("Not HOLDING HOME");
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    m_holdingHomeAlert.setText("Not Running Hold Home");
    Robot.laterator.stop();
  }
}

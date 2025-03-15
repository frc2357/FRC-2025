package frc.robot.commands.util;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PhotonVisionCamera;

public class InitCamera extends Command {

  private PhotonVisionCamera m_cameraToInit;

  public InitCamera(PhotonVisionCamera camToInit) {
    m_cameraToInit = camToInit;
  }

  @Override
  public boolean isFinished() {
    return m_cameraToInit.getCameraMatrix() && m_cameraToInit.getDistCoefs();
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {
    @SuppressWarnings("resource")
    Alert alert = new Alert(
      "CameraAlerts/" + m_cameraToInit.getName(),
      "Camera fully initialized",
      AlertType.kInfo
    );
    alert.set(true);
    if (interrupted) {
      alert.setText("Camera Not Properly Initialized.");
    }
  }
}

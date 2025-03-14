package frc.robot.commands.util;

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
}

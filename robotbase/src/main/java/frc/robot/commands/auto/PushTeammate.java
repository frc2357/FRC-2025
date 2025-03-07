package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.VelDrive;

public class PushTeammate extends AutoBase {

  public PushTeammate() {
    super("Push teammate", "MiddleDitchBranchG");
    m_startTraj.done().onTrue(new VelDrive().withDeadline(new WaitCommand(4)));
  }
}

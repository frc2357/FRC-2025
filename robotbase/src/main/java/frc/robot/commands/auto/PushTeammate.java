package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.LATERATOR;
import frc.robot.commands.drive.VelDrive;
import frc.robot.commands.scoring.coral.CoralHome;
import frc.robot.commands.scoring.coral.CoralPreposeL4;
import frc.robot.commands.scoring.coral.CoralScore;

public class PushTeammate extends AutoBase {

  public PushTeammate() {
    super("Push teammate", "MiddleDitchBranchG");
    m_startTraj.done().onTrue(new VelDrive().withDeadline(new WaitCommand(4)));
  }
}

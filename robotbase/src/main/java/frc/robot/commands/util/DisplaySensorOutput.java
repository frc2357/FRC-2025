package frc.robot.commands.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class DisplaySensorOutput extends Command {

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("LateratorZero", Robot.laterator.isAtZero());
    SmartDashboard.putBoolean("elevatorZero", Robot.elevator.isAtZero());
    SmartDashboard.putBoolean(
      "CoralRunner Intake",
      Robot.coralRunner.isIntakeBeamBroken()
    );
    SmartDashboard.putBoolean(
      "CoralRunner Outtake",
      Robot.coralRunner.isOuttakeBeamBroken()
    );
  }
}

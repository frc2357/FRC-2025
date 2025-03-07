package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

// For helping with tuning velocity drive PID on swerve drive
public class VelDrive extends Command {

  public VelDrive() {
    addRequirements(Robot.swerve);
  }

  @Override
  public void initialize() {
    Robot.swerve.driveFieldRelative(-2, 0, 0);
  }

  @Override
  public void execute() {
    System.out.println(Robot.swerve.getModuleStates()[0].speedMetersPerSecond);

    double rps = Robot.swerve
      .getModule(0)
      .getDriveMotor()
      .getVelocity()
      .getValueAsDouble();
    SmartDashboard.putNumber("Motor rps", rps);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.swerve.stopMotors();
  }
}

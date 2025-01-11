package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.generated.TunerConstants;


public class DefaultDrive extends Command {
  public DefaultDrive() {
    addRequirements(Robot.swerve);
  }

  @Override
  public void execute() {
    double x = Robot.driverControls.getX();
    double y = Robot.driverControls.getY();
    double rotation = Robot.driverControls.getRotation();

    if (x == 0 && y == 0 && rotation == 0) {
      Robot.swerve.stopMotors();
    } else {
      Robot.swerve.driveFieldRelative(
          y * TunerConstants.kSpeedAt12Volts.baseUnitMagnitude(),
          x * TunerConstants.kSpeedAt12Volts.baseUnitMagnitude(),
          rotation * Constants.SWERVE.MAX_ANGULAR_RATE.baseUnitMagnitude());
    }
  }

  @Override
  public void end(boolean interrupted) {
    // Robot.swerve.stopMotors();
  }
}

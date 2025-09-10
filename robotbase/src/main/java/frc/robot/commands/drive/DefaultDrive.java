package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.Constants.SWERVE.TOGGLE_SPEED;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.generated.TunerConstants;

public class DefaultDrive extends Command {

  private static final double m_speedAt12VoltsMPS =
    TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

  public DefaultDrive() {
    addRequirements(Robot.swerve);
  }

  @Override
  public void execute() {
    double x, y, rotation;
    if (SmartDashboard.getBoolean(TOGGLE_SPEED, true)) {
      x = Robot.driverControls.getX() * 0.2;
      y = Robot.driverControls.getY() * 0.2;
      rotation = Robot.driverControls.getRotation() * 0.2;
    } else {
      x = Robot.driverControls.getX();
      y = Robot.driverControls.getY();
      rotation = Robot.driverControls.getRotation();
    }
    if (x == 0 && y == 0 && rotation == 0) {
      Robot.swerve.stopMotors();
    } else {
      Robot.swerve.driveFieldRelative(
        y * m_speedAt12VoltsMPS,
        x * m_speedAt12VoltsMPS,
        rotation * Constants.SWERVE.MAX_ANGULAR_VELOCITY.in(RadiansPerSecond)
      );
    }
  }

  @Override
  public void end(boolean interrupted) {
    // Robot.swerve.stopMotors();
  }
}

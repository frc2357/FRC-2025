package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.generated.TunerConstants;

public class DriveRobotRelative extends Command {

  private static final double m_speedAt12VoltsMPS =
    TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

  public DriveRobotRelative() {
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
      Robot.swerve.driveRobotRelative(
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

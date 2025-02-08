package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FIELD.REEF;
import frc.robot.Constants.SWERVE;
import frc.robot.Robot;
import frc.robot.generated.TunerConstants;

public class DriveLockReef extends Command {

  private static final double m_speedAt12VoltsMPS =
    TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

  public DriveLockReef() {
    addRequirements(Robot.swerve);
  }

  @Override
  public void execute() {
    double x = Robot.driverControls.getX();
    double y = Robot.driverControls.getY();
    double rotation = Robot.driverControls.getRotation();

    if (rotation != 0) {
      Robot.swerve.driveFieldRelative(
        x * m_speedAt12VoltsMPS,
        y * m_speedAt12VoltsMPS,
        rotation * SWERVE.MAX_ANGULAR_VELOCITY.in(RadiansPerSecond)
      );
      return;
    }

    Pose2d relativePose = REEF.CENTER.relativeTo(
      Robot.swerve.getAllianceRelativePose2d()
    );

    Robot.swerve.driveTargetAngle(
      x * m_speedAt12VoltsMPS,
      y * m_speedAt12VoltsMPS,
      new Rotation2d(relativePose.getX(), relativePose.getY())
    );
  }
}

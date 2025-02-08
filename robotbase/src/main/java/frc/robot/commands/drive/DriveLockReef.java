package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
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

    Pose2d relativePose = Robot.swerve
      .getAllianceRelativePose2d()
      .relativeTo(REEF.CENTER);
    Rotation2d calc_rotation = Rotation2d.fromRadians(
      Math.atan2(relativePose.getX(), relativePose.getY())
    );

    Robot.swerve.driveTargetAngle(
      x * m_speedAt12VoltsMPS,
      y * m_speedAt12VoltsMPS,
      calc_rotation
    );
  }
}

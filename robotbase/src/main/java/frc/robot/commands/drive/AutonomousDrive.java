package frc.robot.commands.drive;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class AutonomousDrive extends Command {

  private Timer m_timer;
  private SwerveDrivetrain<TalonFX, TalonFX, CANcoder> m_swerveDrivetrain;
  private final SwerveRequest.FieldCentric m_fieldRelative =
    new SwerveRequest.FieldCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  public AutonomousDrive(
    SwerveDrivetrain<TalonFX, TalonFX, CANcoder> swerveDrivetrain
  ) {
    m_timer = new Timer();
    m_swerveDrivetrain = swerveDrivetrain;
    addRequirements(Robot.swerve);
  }

  @Override
  public void initialize() {
    m_swerveDrivetrain.setControl(
      m_fieldRelative.withVelocityX(-1).withVelocityY(0).withRotationalRate(0)
    );
    m_timer.start();
  }

  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(3);
  }

  @Override
  public void end(boolean interrupted) {
    m_swerveDrivetrain.setControl(
      m_fieldRelative.withVelocityX(0).withVelocityY(0).withRotationalRate(0)
    );
    m_timer.stop();
    m_timer.reset();
  }
}

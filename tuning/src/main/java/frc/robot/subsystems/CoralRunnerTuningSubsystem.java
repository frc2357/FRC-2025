package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN_ID;
import frc.robot.Constants.CORAL_RUNNER;

public class CoralRunnerTuningSubsystem extends SubsystemBase {

  private SparkMax m_motor;

  private double kP = 0;
  private double kI = 0;
  private double kD = 0;
  private double kFF = 0;
  private double maxVel = 0;
  private double maxAcc = 0;

  private SparkBaseConfig m_motorconfig = CORAL_RUNNER.MOTOR_CONFIG;

  private SparkClosedLoopController m_PIDController;
  private RelativeEncoder m_encoder;

  private AngularVelocity m_targetVelocity = Units.RotationsPerSecond.of(
    Double.NaN
  );

  public CoralRunnerTuningSubsystem() {
    m_motor = new SparkMax(CAN_ID.CORAL_RUNNER_MOTOR, MotorType.kBrushless);

    updatePIDs();

    m_motor.configure(
      m_motorconfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );

    m_PIDController = m_motor.getClosedLoopController();
    m_encoder = m_motor.getEncoder();
  }

  public void displayDashboard() {
    SmartDashboard.putNumber("Elevator P", kP);
    SmartDashboard.putNumber("Elevator I", kI);
    SmartDashboard.putNumber("Elevator D", kD);
    SmartDashboard.putNumber("Elevator FF", kFF);
    SmartDashboard.putNumber("Elevator MaxVel", maxVel);
    SmartDashboard.putNumber("Elevator MaxAcc", maxAcc);
    SmartDashboard.putNumber("Elevator Setpoint", 0);
  }

  public void updateDashboard() {
    kP = SmartDashboard.getNumber("Arm P", kP);
    kI = SmartDashboard.getNumber("Arm I", kI);
    kD = SmartDashboard.getNumber("Arm D", kD);
    kFF = SmartDashboard.getNumber("Arm FF", kFF);
    maxVel = SmartDashboard.getNumber("Arm MaxVel", maxVel);
    maxAcc = SmartDashboard.getNumber("Arm MaxAcc", maxAcc);

    SmartDashboard.putNumber("Motor Rotations", m_encoder.getPosition());
    SmartDashboard.putNumber("Velocity (RPM)", getVelocity().magnitude());

    updatePIDs();
  }

  public void updatePIDs() {
    m_motorconfig.closedLoop.pidf(kP, kI, kD, kFF);

    m_motorconfig.closedLoop.maxMotion
      .maxAcceleration(maxAcc)
      .maxVelocity(maxVel);
  }

  public void teleopPeriodic() {
    double velocityTarget;
    velocityTarget = SmartDashboard.getNumber(
      "CoralRunner Velocity (RPM) Target",
      0
    );
    setTargetVelocity(Units.RotationsPerSecond.of(velocityTarget));
    // }
  }

  public void setTargetVelocity(AngularVelocity targetVelocity) {
    m_targetVelocity = targetVelocity;
    m_PIDController.setReference(
      targetVelocity.in(RPM),
      ControlType.kMAXMotionVelocityControl
    );
  }

  public AngularVelocity getVelocity() {
    return RotationsPerSecond.of(m_encoder.getVelocity());
  }

  public Boolean isAtTargetVelocity() {
    return m_targetVelocity.isNear(
      getVelocity(),
      CORAL_RUNNER.MAX_MOTION_ALLOWED_ERROR_PERCENT
    );
  }

  public void setSpeed(double percentOutput) {
    m_targetVelocity = RotationsPerSecond.of(Double.NaN);
    m_motor.set(percentOutput);
  }

  public void setAxisSpeed(double axisSpeed) {
    m_targetVelocity = RotationsPerSecond.of(Double.NaN);
    axisSpeed *= CORAL_RUNNER.AXIS_MAX_SPEED;
    m_motor.set(axisSpeed);
  }

  public void stop() {
    m_targetVelocity = RotationsPerSecond.of(Double.NaN);
    m_motor.stopMotor();
  }
}

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ELEVATOR;

public class ElevatorTuningSubsystem {

  private SparkMax m_motorLeft;
  private SparkMax m_motorRight;
  private SparkClosedLoopController m_PIDController;
  private RelativeEncoder m_encoder;
  private Angle m_targetRotations = Units.Rotations.of(Double.NaN);

  private double kP = 0;
  private double kI = 0;
  private double kD = 0;
  private double kFF = 0;
  private double maxVel = 0;
  private double maxAcc = 0;

  private SparkBaseConfig m_motorconfig = Constants.ELEVATOR.MOTOR_CONFIG_LEFT;

  // private PIDController m_PidController

  public ElevatorTuningSubsystem() {
    m_motorLeft = new SparkMax(
      Constants.CAN_ID.ELEVATOR_LEFT_MOTOR,
      MotorType.kBrushless
    );
    m_motorRight = new SparkMax(
      Constants.CAN_ID.ELEVATOR_RIGHT_MOTOR,
      MotorType.kBrushless
    );

    m_motorLeft.configure(
      m_motorconfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );
    m_encoder = m_motorLeft.getEncoder();

    m_motorRight.configure(
      Constants.ELEVATOR.MOTOR_CONFIG_RIGHT,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );

    m_PIDController = m_motorLeft.getClosedLoopController();
    displayDashboard();
  }

  public void updatePIDs() {
    m_motorconfig.closedLoop.pidf(kP, kI, kD, kFF);

    m_motorconfig.closedLoop.maxMotion
      .maxAcceleration(maxAcc)
      .maxVelocity(maxVel);
  }

  public void displayDashboard() {
    SmartDashboard.putNumber("Elevator P", kP);
    SmartDashboard.putNumber("Elevator I", kI);
    SmartDashboard.putNumber("Elevator D", kD);
    SmartDashboard.putNumber("Elevator FF", kFF);
    SmartDashboard.putNumber("Elevator MaxVel", maxVel);
    SmartDashboard.putNumber("Elevator MaxAcc", maxAcc);
    SmartDashboard.putNumber("Motor Rotations", m_encoder.getPosition());
    SmartDashboard.putBoolean("Is At Target", isAtTarget());
    SmartDashboard.putNumber("Calculated Distance", getDistance().magnitude());
    SmartDashboard.putNumber("Elevator Setpoint", m_encoder.getPosition());
  }

  public void updateDashboard() {
    kP = SmartDashboard.getNumber("Elevator P", kP);
    kI = SmartDashboard.getNumber("Elevator I", kI);
    kD = SmartDashboard.getNumber("Elevator D", kD);
    kFF = SmartDashboard.getNumber("Elevator FF", kFF);
    maxVel = SmartDashboard.getNumber("Elevator MaxVel", maxVel);
    maxAcc = SmartDashboard.getNumber("Elevator MaxAcc", maxAcc);
    SmartDashboard.putNumber("Motor Rotations", m_encoder.getPosition());
    SmartDashboard.putBoolean("Is At Target", isAtTarget());
    SmartDashboard.putNumber("Calculated Distance", getDistance().magnitude());

    updatePIDs();
  }

  public void teleopPeriodic() {
    // if (SmartDashboard.getBoolean("UseDistance", false)) {

    // double distanceSetpoint;
    // distanceSetpoint = SmartDashboard.getNumber("Elevator Distance Setpoint", 0);
    // setTargetDistance(Units.Feet.of(distanceSetpoint));
    // } else {

    double rotationSetpoint;
    rotationSetpoint = SmartDashboard.getNumber("Elevator Setpoint", 0);
    System.out.println(rotationSetpoint);
    setTargetRotations(Degrees.of(rotationSetpoint));
    // }

  }

  public void setSpeed(double speed) {
    m_motorLeft.set(speed);
    m_targetRotations = Units.Rotations.of(Double.NaN);
  }

  public void stop() {
    m_motorLeft.stopMotor();
    m_targetRotations = Units.Rotations.of(Double.NaN);
  }

  public AngularVelocity getVelocity() {
    return Units.RotationsPerSecond.of(m_encoder.getVelocity() / 60);
  }

  public void setZero() {
    m_encoder.setPosition(0);
  }

  private Angle getRotations() {
    return Units.Rotations.of(m_encoder.getPosition());
  }

  public Distance getDistance() {
    return (
      ELEVATOR.MOTOR_PULLEY_PITCH_DIAMETER.times(m_encoder.getPosition())
    );
  }

  private void setTargetRotations(Angle targetRotations) {
    m_targetRotations = targetRotations;
    m_PIDController.setReference(
      m_targetRotations.in(Units.Rotations),
      ControlType.kMAXMotionPositionControl
    );
    System.out.println(m_PIDController);
    System.out.println(m_targetRotations);
  }

  public void setTargetDistance(Distance targetDistance) {
    Angle rotations = Units.Rotations.of(
      targetDistance.div(ELEVATOR.MOTOR_PULLEY_PITCH_DIAMETER).magnitude()
    );
    setTargetRotations(rotations);
  }

  private boolean isAtTargetRotations() {
    return m_targetRotations.isNear(
      getRotations(),
      ELEVATOR.MAX_MOTION_ALLOWED_ERROR_PERCENT
    );
  }

  public boolean isAtTarget() {
    return isAtTargetRotations();
  }

  public void setAxisSpeed(double speed) {
    m_targetRotations = Units.Rotations.of(Double.NaN);
    speed *= ELEVATOR.AXIS_MAX_SPEED;
    m_motorLeft.set(speed);
  }
}

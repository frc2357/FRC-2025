package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Feet;

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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CAN_ID;
import frc.robot.Constants.DIGITAL_INPUT;
import frc.robot.Constants.LATERATOR;

public class LateratorTuningSubsystem {

  private SparkMax m_motorLeft;
  private SparkMax m_motorRight;

  private DigitalInput m_hallEffectSensor;

  private double kP = 0;
  private double kI = 0;
  private double kD = 0;
  private double kFF = 0;
  private double maxVel = 0;
  private double maxAcc = 0;

  private SparkBaseConfig motorconfig = LATERATOR.MOTOR_CONFIG_LEFT;

  private SparkClosedLoopController m_PIDController;
  private RelativeEncoder m_encoder;

  private Angle m_targetRotations = Units.Rotations.of(Double.NaN);

  public LateratorTuningSubsystem() {
    m_motorLeft = new SparkMax(
      CAN_ID.LATERATOR_MOTOR_LEFT,
      MotorType.kBrushless
    );

    m_motorRight = new SparkMax(
      CAN_ID.LATERATOR_MOTOR_RIGHT,
      MotorType.kBrushless
    );

    updatePIDs();

    m_motorLeft.configure(
      motorconfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );

    m_motorRight.configure(
      LATERATOR.MOTOR_CONFIG_RIGHT,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );

    m_PIDController = m_motorLeft.getClosedLoopController();

    m_encoder = m_motorLeft.getEncoder();

    m_hallEffectSensor = new DigitalInput(
      DIGITAL_INPUT.LATERATOR_CENTER_HALL_EFFECT_SENSOR_ID
    );
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
    SmartDashboard.putBoolean("Is At Zero", isAtZero());
    SmartDashboard.putBoolean("Is At Target", isAtTarget());
    SmartDashboard.putNumber("Calculated Distance", getPosition().magnitude());

    updatePIDs();
  }

  public void updatePIDs() {
    motorconfig.closedLoop.pidf(kP, kI, kD, kFF);

    motorconfig.closedLoop.maxMotion
      .maxAcceleration(maxAcc)
      .maxVelocity(maxVel);
  }

  public void teleopPeriodic() {
    double rotationSetpoint;
    rotationSetpoint = SmartDashboard.getNumber(
      "Laterator Rotation Setpoint",
      0
    );
    setTargetRotations(Units.Degrees.of(rotationSetpoint));
    // }
  }

  private void setTargetRotations(Angle targetRotations) {
    m_targetRotations = targetRotations;
    m_PIDController.setReference(
      m_targetRotations.in(Units.Rotations),
      ControlType.kMAXMotionPositionControl
    );
  }

  public void setTargetDistance(Distance targetDistance) {
    Angle rotations = Units.Rotations.of(targetDistance.in(Feet)); //TODO: Add accurate conversion information
    setTargetRotations(rotations);
  }

  private Angle getRotations() {
    return Units.Rotations.of(m_encoder.getPosition());
  }

  private boolean isAtTargetRotations() {
    return m_targetRotations.isNear(
      getRotations(),
      LATERATOR.MAX_MOTION_ALLOWED_ERROR_PERCENT
    );
  }

  public boolean isAtTarget() {
    return isAtTargetRotations();
  }

  public boolean isAtZero() {
    return m_hallEffectSensor.get();
  }

  public AngularVelocity getVelocity() {
    return Units.RotationsPerSecond.of(m_encoder.getVelocity() / 60);
  }

  public Distance getPosition() {
    return Feet.of(0); //TODO: Add accurate conversion information
  }

  public void setZero() {
    m_encoder.setPosition(0);
  }

  public void setSpeed(double speed) {
    m_motorLeft.set(speed);
    m_targetRotations = Units.Rotations.of(Double.NaN);
  }

  public void setAxisSpeed(double axisSpeed) {
    axisSpeed *= LATERATOR.AXIS_MAX_SPEED;
    m_motorLeft.set(axisSpeed);
    m_targetRotations = Units.Rotations.of(Double.NaN);
  }

  public void stop() {
    m_motorLeft.stopMotor();
    m_targetRotations = Units.Rotations.of(Double.NaN);
  }
}

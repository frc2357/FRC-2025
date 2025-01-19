package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN_ID;
import frc.robot.Constants.DIGITAL_INPUT;
import frc.robot.Constants.LATERATOR;

public class Laterator extends SubsystemBase {

  private SparkMax m_motor;

  private DigitalInput m_hallEffectSensor;

  private SparkClosedLoopController m_PIDController;
  private RelativeEncoder m_encoder;

  private Angle m_targetRotations = Units.Rotations.of(Double.NaN);

  public Laterator() {
    m_motor = new SparkMax(CAN_ID.LATERATOR_MOTOR, MotorType.kBrushless);
    m_motor.configure(
      LATERATOR.MOTOR_CONFIG,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );

    m_PIDController = m_motor.getClosedLoopController();

    m_encoder = m_motor.getEncoder();

    m_hallEffectSensor = new DigitalInput(
      DIGITAL_INPUT.LATERATOR_CENTER_HALL_EFFECT_SENSOR_ID
    );
  }

  public void setTargetRotations(Angle targetRotations) {
    m_targetRotations = targetRotations;
    m_PIDController.setReference(
      m_targetRotations.in(Units.Rotations),
      ControlType.kMAXMotionPositionControl
    );
  }

  public Angle getRotations() {
    return Units.Rotations.of(m_encoder.getPosition());
  }

  public boolean isAtTargetRotations() {
    return m_targetRotations.isNear(
      getRotations(),
      LATERATOR.MAX_MOTION_ALLOWED_ERROR_PERCENT
    );
  }

  public boolean isAtSetpoint() {
    return isAtTargetRotations();
  }

  public boolean isAtCenter() {
    return m_hallEffectSensor.get();
  }

  public AngularVelocity getVelocity() {
    return Units.RotationsPerSecond.of(m_encoder.getVelocity() / 60);
  }

  public void setZero() {
    m_encoder.setPosition(0);
  }

  public void setSpeed(double speed) {
    m_motor.set(speed);
  }

  public void setAxisSpeed(double axisSpeed) {
    axisSpeed *= LATERATOR.AXIS_MAX_SPEED;
    m_motor.set(axisSpeed);
  }
}

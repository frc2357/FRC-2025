package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ELEVATOR;

public class Elevator extends SubsystemBase {

  private SparkMax m_motorLeft;
  private SparkMax m_motorRight;
  private SparkClosedLoopController m_PIDController;
  private RelativeEncoder m_encoder;
  private MutAngle m_targetRotations = Units.Rotations.mutable(Double.NaN);

  public Elevator() {
    m_motorLeft = new SparkMax(
      Constants.CAN_ID.ELEVATOR_LEFT_MOTOR,
      MotorType.kBrushless
    );
    m_motorRight = new SparkMax(
      Constants.CAN_ID.ELEVATOR_RIGHT_MOTOR,
      MotorType.kBrushless
    );

    m_motorLeft.configure(
      Constants.ELEVATOR.MOTOR_CONFIG_LEFT,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );
    m_motorRight.configure(
      Constants.ELEVATOR.MOTOR_CONFIG_RIGHT,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );

    m_PIDController = m_motorLeft.getClosedLoopController();

    m_encoder = m_motorLeft.getEncoder();
  }

  public void setSpeed(double speed) {
    m_motorLeft.set(speed);
    m_targetRotations.mut_replace(Double.NaN, Rotations);
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

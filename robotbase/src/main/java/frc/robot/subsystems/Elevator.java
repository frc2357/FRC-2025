package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ELEVATOR;

public class Elevator extends SubsystemBase {

  private DigitalInput m_hall_effect;
  private Debouncer m_debouncer;
  private SparkMax m_motorLeft;
  private SparkMax m_motorRight;
  private RelativeEncoder m_encoder;

  private MutDistance m_targetDistance = Units.Meters.mutable(Double.NaN);
  private MutLinearVelocity m_currentLinearVelocityHolder =
    Units.MetersPerSecond.mutable(Double.NaN);
  private MutAngle m_currentRotationsHolder = Units.Rotations.mutable(
    Double.NaN
  );

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

    m_encoder = m_motorLeft.getEncoder();

    m_hall_effect = new DigitalInput(
      Constants.DIGITAL_INPUT.ELEVATOR_CENTER_HALL_EFFECT_SENSOR_ID
    );
    m_debouncer = new Debouncer(Constants.ELEVATOR.DEBOUNCE_TIME_SECONDS);
  }

  public void setSpeed(double percentOutput) {
    m_motorLeft.set(percentOutput);
    m_targetDistance.mut_replace(Double.NaN, Units.Meters);
  }

  public void setAxisSpeed(double speed) {
    speed *= ELEVATOR.AXIS_MAX_SPEED;
    m_motorLeft.set(speed);
    m_targetDistance.mut_replace(Double.NaN, Units.Meters);
  }

  public void stop() {
    m_motorLeft.stopMotor();
    m_targetDistance.mut_replace(Double.NaN, Units.Meters);
  }

  public void reachDistance(Distance targetDistance) {
    m_targetDistance.mut_replace(targetDistance);
    double volts = MathUtil.clamp(
      ELEVATOR.PID_CONTROLLER.calculate(
        getDistance().in(Units.Meters),
        m_targetDistance.in(Units.Meters)
      ) +
      ELEVATOR.FEEDFORWARD.calculateWithVelocities(
        getVelocity().in(Units.MetersPerSecond),
        ELEVATOR.PID_CONTROLLER.getSetpoint().velocity
      ),
      -ELEVATOR.MAX_VOLTS,
      ELEVATOR.MAX_VOLTS
    );

    m_motorLeft.setVoltage(volts);
  }

  public LinearVelocity getVelocity() {
    double vel =
      ((m_encoder.getVelocity() / 60) / ELEVATOR.GEAR_RATIO) *
      (ELEVATOR.OUTPUT_PULLEY_CIRCUMFERENCE.in(Units.Meters));
    m_currentLinearVelocityHolder.mut_replace(vel, Units.MetersPerSecond);
    return m_currentLinearVelocityHolder;
  }

  private Angle getRotations() {
    m_currentRotationsHolder.mut_replace(
      m_encoder.getPosition(),
      Units.Rotations
    );
    return m_currentRotationsHolder;
  }

  public Distance getDistance() {
    return (
      ELEVATOR.OUTPUT_PULLEY_CIRCUMFERENCE.times(
        getRotations().div(ELEVATOR.GEAR_RATIO).in(Units.Rotations)
      )
    );
  }

  private boolean isAtTargetRotations() {
    return m_targetDistance.isNear(
      getRotations(),
      ELEVATOR.ALLOWED_POSITION_ERROR_PERCENT
    );
  }

  public boolean isAtTarget() {
    return isAtTargetRotations();
  }

  public boolean isAtZero() {
    return m_debouncer.calculate(!m_hall_effect.get());
  }

  public void setZero() {
    m_encoder.setPosition(0);
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("Elevator RPM", m_encoder.getVelocity());
  }
}

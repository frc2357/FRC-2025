package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN_ID;
import frc.robot.Constants.CORAL_RUNNER;
import frc.robot.Constants.DIGITAL_INPUT;

public class CoralRunner extends SubsystemBase {

  private SparkMax m_motor;

  private DigitalInput m_beamBreakIntake;
  private DigitalInput m_beamBreakOuttake;
  private Debouncer m_debouncerOuttake;
  private Debouncer m_debouncerIntake;
  private boolean m_isIntakeBeamBroken;
  private boolean m_isOutakeBeamBroken;
  private RelativeEncoder m_encoder;

  private MutAngularVelocity m_currentVelocityHolder = Units.RPM.mutable(
    Double.NaN
  );

  public CoralRunner() {
    m_motor = new SparkMax(CAN_ID.CORAL_RUNNER_MOTOR, MotorType.kBrushless);
    m_motor.configure(
      CORAL_RUNNER.MOTOR_CONFIG,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );

    m_encoder = m_motor.getEncoder();

    m_debouncerIntake = new Debouncer(
      CORAL_RUNNER.DEBOUNCE_TIME_SECONDS,
      DebounceType.kBoth
    );

    m_debouncerOuttake = new Debouncer(
      CORAL_RUNNER.DEBOUNCE_TIME_SECONDS,
      DebounceType.kBoth
    );

    m_beamBreakIntake = new DigitalInput(
      DIGITAL_INPUT.CORAL_RUNNER_BEAM_BREAK_INTAKE_ID
    );
    m_beamBreakOuttake = new DigitalInput(
      DIGITAL_INPUT.CORAL_RUNNER_BEAM_BREAK_OUTTAKE_ID
    );
  }

  public void setSpeed(Dimensionless percent) {
    m_motor.set(percent.in(Units.Percent));
  }

  public void setAxisSpeed(double axisSpeed) {
    axisSpeed *= CORAL_RUNNER.AXIS_MAX_SPEED;
    m_motor.set(axisSpeed);
  }

  public void stop() {
    m_motor.stopMotor();
  }

  public AngularVelocity getVelocity() {
    m_currentVelocityHolder.mut_replace(m_encoder.getVelocity(), Units.RPM);
    return m_currentVelocityHolder;
  }

  public boolean isIntakeBeamBroken() {
    return m_isIntakeBeamBroken;
  }

  public boolean isOuttakeBeamBroken() {
    return m_isOutakeBeamBroken;
  }

  public boolean isStalling() {
    return m_motor.getOutputCurrent() > CORAL_RUNNER.STALL_AMPS;
  }

  public boolean containsCoral() {
    return m_isIntakeBeamBroken || m_isOutakeBeamBroken;
  }

  public boolean hasNoCoral() {
    return !m_isIntakeBeamBroken && !m_isOutakeBeamBroken;
  }

  @Override
  public void periodic() {
    m_isIntakeBeamBroken = m_debouncerOuttake.calculate(
      !m_beamBreakIntake.get()
    );
    m_isOutakeBeamBroken = m_debouncerIntake.calculate(
      !m_beamBreakOuttake.get()
    );
  }
}

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN_ID;
import frc.robot.Constants.CORAL_RUNNER;
import frc.robot.Constants.DIGITAL_INPUT;

public class CoralRunner extends SubsystemBase {

  private SparkMax m_motor;

  private DigitalInput m_beamBreakIntake;
  private DigitalInput m_beamBreakOuttake;
  private Debouncer m_debouncer;

  public CoralRunner() {
    m_motor = new SparkMax(CAN_ID.CORAL_RUNNER_MOTOR, MotorType.kBrushless);
    m_motor.configure(
      CORAL_RUNNER.MOTOR_CONFIG,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );

    m_debouncer = new Debouncer(
      CORAL_RUNNER.DEBOUNCE_TIME_SECONDS,
      DebounceType.kBoth
    );

    m_beamBreakIntake = new DigitalInput(
      DIGITAL_INPUT.CORAL_RUNNER_BEAM_BREAK_INTAKE_ID
    );
    m_beamBreakIntake = new DigitalInput(
      DIGITAL_INPUT.CORAL_RUNNER_BEAM_BREAK_OUTTAKE_ID
    );
  }

  public void set(double percentOutput) {
    m_motor.set(percentOutput);
  }

  public void setAxisSpeed(double axisSpeed) {
    axisSpeed *= CORAL_RUNNER.AXIS_MAX_SPEED;
    m_motor.set(axisSpeed);
  }

  public void stop() {
    m_motor.stopMotor();
  }

  public boolean isIntakeBeamBroken() {
    return m_debouncer.calculate(m_beamBreakIntake.get());
  }

  public boolean isOuttakeBeamBroken() {
    return m_debouncer.calculate(m_beamBreakOuttake.get());
  }
}

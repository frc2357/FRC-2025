package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ALGAE_PIVOT;
import frc.robot.Constants.CAN_ID;

public class AlgaePivotTuningSubsystem extends SubsystemBase {

  private Angle m_targetAngle;
  private SparkMax m_leftMotor;
  private SparkMax m_rightMotor;
  private SparkAbsoluteEncoder m_absoluteEncoder;
  private SparkClosedLoopController m_leftPidController;

  private double kP = 0;
  private double kI = 0;
  private double kD = 0;
  private double kFF = 0;
  private double maxVel = 0;
  private double maxAcc = 0;

  private SparkBaseConfig motorConfig = ALGAE_PIVOT.LEFT_MOTOR_CONFIG;

  public AlgaePivotTuningSubsystem() {
    m_leftMotor = new SparkMax(
      CAN_ID.LEFT_ALGAE_PIVOT_MOTOR,
      MotorType.kBrushless
    );
    m_rightMotor = new SparkMax(
      CAN_ID.RIGHT_ALGAE_PIVOT_MOTOR,
      MotorType.kBrushless
    );

    updatePIDs();

    m_rightMotor.configure(
      Constants.ALGAE_PIVOT.RIGHT_MOTOR_CONFIG,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );

    m_leftMotor.configure(
      motorConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );

    m_leftPidController = m_leftMotor.getClosedLoopController();
    displayDashboard();
  }

  public void updatePIDs() {
    motorConfig.closedLoop.pidf(kP, kI, kD, kFF);

    motorConfig.closedLoop.maxMotion
      .maxAcceleration(maxAcc)
      .maxVelocity(maxVel);
  }

  public void displayDashboard() {
    SmartDashboard.putNumber("AlgaePivot P", kP);
    SmartDashboard.putNumber("AlgaePivot I", kI);
    SmartDashboard.putNumber("AlgaePivot D", kD);
    SmartDashboard.putNumber("AlgaePivot FF", kFF);
    SmartDashboard.putNumber("AlgaePivot MaxVel", maxVel);
    SmartDashboard.putNumber("AlgaePivot MaxAcc", maxAcc);
    SmartDashboard.putNumber("AlgaePivot Degree Setpoint", 0);
  }

  public void updateDashboard() {
    kP = SmartDashboard.getNumber("Arm P", kP);
    kI = SmartDashboard.getNumber("Arm I", kI);
    kD = SmartDashboard.getNumber("Arm D", kD);
    kFF = SmartDashboard.getNumber("Arm FF", kFF);
    maxVel = SmartDashboard.getNumber("Arm MaxVel", maxVel);
    maxAcc = SmartDashboard.getNumber("Arm MaxAcc", maxAcc);

    SmartDashboard.putNumber("Motor Degrees", getAngle().in(Units.Degrees));
    SmartDashboard.putBoolean("Is At Setpoint", isAtTargetAngle());

    updatePIDs();
  }

  public void teleopPeriodic() {
    updateDashboard();
    double angleSetpoint = SmartDashboard.getNumber(
      "AlgaePivot Degree Setpoint",
      0
    );
    setTargetAngle(Units.Degrees.of(angleSetpoint));
  }

  public void setAxisSpeed(double axisSpeed) {
    axisSpeed *= ALGAE_PIVOT.AXIS_MAX_SPEED;
    set(axisSpeed);
  }

  public void set(double percentOutput) {
    m_targetAngle = Units.Degree.of(Double.NaN);
    m_leftMotor.set(percentOutput);
  }

  public Angle getAngle() {
    return Units.Degree.of(m_absoluteEncoder.getPosition() * 360);
  }

  public Angle getTargetAngle() {
    return m_targetAngle;
  }

  public void setTargetAngle(Angle angle) {
    if (angle.lt(ALGAE_PIVOT.MIN_ANGLE)) {
      System.err.println("ALGAE PIVOT: Cannot set angle lower than minimum");
      return;
    }
    if (angle.gt(ALGAE_PIVOT.MAX_ANGLE)) {
      System.err.println("ALGAE PIVOT: Cannot set angle higher than minimum");
      return;
    }

    m_targetAngle = angle;
    m_leftPidController.setReference(
      angle.in(Units.Rotations),
      ControlType.kPosition
    );
  }

  public boolean isAtTargetAngle() {
    return m_targetAngle.isNear(
      getAngle(),
      ALGAE_PIVOT.MAX_MOTION_ALLOWED_ERROR_PERCENT
    );
  }

  public void stop() {
    m_leftMotor.stopMotor();
    m_targetAngle = Units.Degree.of(Double.NaN);
  }
}

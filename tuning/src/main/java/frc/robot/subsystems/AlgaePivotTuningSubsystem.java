package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ALGAE_PIVOT;

// TODO: Goal: Unknown
public class AlgaePivotTuningSubsystem implements Sendable {

  private SparkMax m_motorLeft;
  private SparkMax m_motorRight;
  private SparkClosedLoopController m_PIDController;
  private SparkAbsoluteEncoder m_absoluteEncoder;
  private Angle m_targetRotations = Units.Rotations.of(Double.NaN);

  private double P = 0.0;
  private double I = 0;
  private double D = 0;
  private double arbFF = 0.0;
  private double maxVel = 1000; // Desired: 4600, Max: 5600
  private double maxAcc = 500; // Desired: 18400

  private SparkBaseConfig m_motorconfig =
    Constants.ALGAE_PIVOT.MOTOR_CONFIG_RIGHT;

  public AlgaePivotTuningSubsystem() {
    m_motorLeft = new SparkMax(
      Constants.CAN_ID.ALGAE_PIVOT_LEFT_MOTOR,
      MotorType.kBrushless
    );
    m_motorRight = new SparkMax(
      Constants.CAN_ID.ALGAE_PIVOT_RIGHT_MOTOR,
      MotorType.kBrushless
    );

    m_motorLeft.configure(
      Constants.ALGAE_PIVOT.MOTOR_CONFIG_LEFT,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );

    m_motorRight.configure(
      m_motorconfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );

    m_absoluteEncoder = m_motorRight.getAbsoluteEncoder();

    m_PIDController = m_motorRight.getClosedLoopController();

    Preferences.initDouble("algaePivotP", 0);
    Preferences.initDouble("algaePivotI", 0);
    Preferences.initDouble("algaePivotD", 0);
    Preferences.initDouble("algaePivotFF", 0);
    Preferences.initDouble("algaePivotMaxVel", 0);
    Preferences.initDouble("algaePivotMaxAcc", 0);

    P = Preferences.getDouble("algaePivotP", 0);
    I = Preferences.getDouble("algaePivotI", 0);
    D = Preferences.getDouble("algaePivotD", 0);
    arbFF = Preferences.getDouble("algaePivotFF", 0);
    maxVel = Preferences.getDouble("algaePivotMaxVel", 0);
    maxAcc = Preferences.getDouble("algaePivotMaxAcc", 0);

    displayDashboard();
  }

  public void displayDashboard() {
    SmartDashboard.putNumber("Algae Pivot P", P);
    SmartDashboard.putNumber("Algae Pivot I", I);
    SmartDashboard.putNumber("Algae Pivot D", D);
    SmartDashboard.putNumber("Algae Pivot arbFF", arbFF);
    SmartDashboard.putNumber("Algae Pivot MaxVel", maxVel);
    SmartDashboard.putNumber("Algae Pivot MaxAcc", maxAcc);
    SmartDashboard.putNumber(
      "Motor Rotations",
      m_absoluteEncoder.getPosition()
    );
    SmartDashboard.putNumber("Motor Velocity", m_absoluteEncoder.getVelocity());
    SmartDashboard.putBoolean("Is At Target", isAtTargetAngle());
    SmartDashboard.putNumber("Algae Pivot Setpoint", 0);
    SmartDashboard.putData("Save Algae Pivot Config", this);
  }

  public void updatePIDs() {
    // Rev recommends not using velocity feed forward for max motion positional control
    m_motorconfig.closedLoop.pidf(P, I, D, 0);

    m_motorconfig.closedLoop.maxMotion
      .maxAcceleration(maxAcc)
      .maxVelocity(maxVel);

    m_motorRight.configure(
      m_motorconfig,
      ResetMode.kNoResetSafeParameters,
      PersistMode.kNoPersistParameters
    );
  }

  public void updateDashboard() {
    double newP = SmartDashboard.getNumber("Algae Pivot P", P);
    double newI = SmartDashboard.getNumber("Algae Pivot I", I);
    double newD = SmartDashboard.getNumber("Algae Pivot D", D);
    double newFF = SmartDashboard.getNumber("Algae Pivot arbFF", arbFF);
    double newMaxVel = SmartDashboard.getNumber("Algae Pivot MaxVel", maxVel);
    double newMaxAcc = SmartDashboard.getNumber("Algae Pivot MaxAcc", maxAcc);
    SmartDashboard.putNumber(
      "Motor Rotations",
      m_absoluteEncoder.getPosition()
    );
    SmartDashboard.putNumber("Motor Velocity", m_absoluteEncoder.getVelocity());
    m_targetRotations = Rotations.of(
      SmartDashboard.getNumber("Algae Pivot Setpoint", 0)
    );
    SmartDashboard.putBoolean("Is At Target", isAtTargetAngle());

    if (
      newP != P ||
      newI != I ||
      newD != D ||
      newFF != arbFF ||
      newMaxVel != maxVel ||
      newMaxAcc != maxAcc
    ) {
      P = newP;
      I = newI;
      D = newD;
      arbFF = newFF;
      maxVel = newMaxVel;
      maxAcc = newMaxAcc;
      updatePIDs();
    }
  }

  public void teleopPeriodic() {
    // if (SmartDashboard.getBoolean("UseDistance", false)) {

    // double distanceSetpoint;
    // distanceSetpoint = SmartDashboard.getNumber("Algae Pivot Distance Setpoint", 0);
    // setTargetDistance(Units.Feet.of(distanceSetpoint));
    // } else {

    setTargetAngle(m_targetRotations);
    // }

  }

  public void setSpeed(double speed) {
    m_motorRight.set(speed);
  }

  public void setAxisSpeed(double speed) {
    m_targetRotations = Units.Rotations.of(Double.NaN);
    speed *= ALGAE_PIVOT.AXIS_MAX_SPEED;
    m_motorRight.set(speed);
  }

  public void stop() {
    m_motorRight.stopMotor();
  }

  public AngularVelocity getVelocity() {
    return Units.RPM.of(m_absoluteEncoder.getVelocity());
  }

  public Angle getAngle() {
    return Units.Rotations.of(m_absoluteEncoder.getPosition());
  }

  private void setTargetAngle(Angle targetRotations) {
    m_targetRotations = targetRotations;
    m_PIDController.setReference(
      m_targetRotations.in(Units.Rotations),
      ControlType.kMAXMotionPositionControl,
      ClosedLoopSlot.kSlot0,
      arbFF,
      ArbFFUnits.kVoltage
    );
  }

  private boolean isAtTargetAngle() {
    return m_targetRotations.isNear(
      getAngle(),
      ALGAE_PIVOT.MAX_MOTION_ALLOWED_ERROR_PERCENT
    );
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("algaePivot");

    builder.addBooleanProperty(
      "Save Config",
      () -> false,
      value -> {
        Preferences.setDouble("algaePivotP", P);
        Preferences.setDouble("algaePivotI", I);
        Preferences.setDouble("algaePivotD", D);
        Preferences.setDouble("algaePivotFF", arbFF);
        Preferences.setDouble("algaePivotMaxVel", maxVel);
        Preferences.setDouble("algaePivotMaxAcc", maxAcc);
      }
    );
  }
}

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
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
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ELEVATOR;

// TODO: Goal: Full elevator extension and retraction in 0.5 seconds
public class ElevatorTuningSubsystem implements Sendable {

  private SparkMax m_motorLeft;
  private SparkMax m_motorRight;
  private SparkClosedLoopController m_PIDController;
  private RelativeEncoder m_encoder;
  private Angle m_targetRotations = Units.Rotations.of(Double.NaN);

  private double P = 0.008;
  private double I = 0;
  private double D = 0;
  private double arbFF = 0.05;
  private double maxVel = 4600; // Desired: 4600, Max: 5600
  private double maxAcc = 5000; // Desired: 18400

  private SparkBaseConfig m_motorconfig = Constants.ELEVATOR.MOTOR_CONFIG_LEFT;

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

    Preferences.initDouble("elevatorP", 0);
    Preferences.initDouble("elevatorI", 0);
    Preferences.initDouble("elevatorD", 0);
    Preferences.initDouble("elevatorFF", 0);
    Preferences.initDouble("elevatorMaxVel", 0);
    Preferences.initDouble("elevatorMaxAcc", 0);

    P = Preferences.getDouble("elevatorP", 0);
    I = Preferences.getDouble("elevatorI", 0);
    D = Preferences.getDouble("elevatorD", 0);
    arbFF = Preferences.getDouble("elevatorFF", 0);
    maxVel = Preferences.getDouble("elevatorMaxVel", 0);
    maxAcc = Preferences.getDouble("elevatorMaxAcc", 0);

    displayDashboard();
  }

  public void displayDashboard() {
    SmartDashboard.putNumber("Elevator P", P);
    SmartDashboard.putNumber("Elevator I", I);
    SmartDashboard.putNumber("Elevator D", D);
    SmartDashboard.putNumber("Elevator arbFF", arbFF);
    SmartDashboard.putNumber("Elevator MaxVel", maxVel);
    SmartDashboard.putNumber("Elevator MaxAcc", maxAcc);
    SmartDashboard.putNumber("Motor Rotations", m_encoder.getPosition());
    SmartDashboard.putNumber("Motor Velocity", m_encoder.getVelocity());
    SmartDashboard.putBoolean("Is At Target", isAtTargetRotations());
    SmartDashboard.putNumber("Calculated Distance", getDistance().magnitude());
    SmartDashboard.putNumber("Elevator Setpoint", 0);
    SmartDashboard.putData("Save Elevator Config", this);
  }

  public void updatePIDs() {
    // Rev recommends not using velocity feed forward for max motion positional control
    m_motorconfig.closedLoop.pidf(P, I, D, 0);

    m_motorconfig.closedLoop.maxMotion
      .maxAcceleration(maxAcc)
      .maxVelocity(maxVel);

    m_motorLeft.configure(
      m_motorconfig,
      ResetMode.kNoResetSafeParameters,
      PersistMode.kNoPersistParameters
    );
  }

  public void updateDashboard() {
    double newP = SmartDashboard.getNumber("Elevator P", P);
    double newI = SmartDashboard.getNumber("Elevator I", I);
    double newD = SmartDashboard.getNumber("Elevator D", D);
    double newFF = SmartDashboard.getNumber("Elevator arbFF", arbFF);
    double newMaxVel = SmartDashboard.getNumber("Elevator MaxVel", maxVel);
    double newMaxAcc = SmartDashboard.getNumber("Elevator MaxAcc", maxAcc);
    SmartDashboard.putNumber("Motor Rotations", m_encoder.getPosition());
    SmartDashboard.putNumber("Motor Velocity", m_encoder.getVelocity());
    m_targetRotations = Rotations.of(
      SmartDashboard.getNumber("Elevator Setpoint", 0)
    );
    SmartDashboard.putBoolean("Is At Target", isAtTargetRotations());
    SmartDashboard.putNumber("Calculated Distance", getDistance().magnitude());

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
    // distanceSetpoint = SmartDashboard.getNumber("Elevator Distance Setpoint", 0);
    // setTargetDistance(Units.Feet.of(distanceSetpoint));
    // } else {

    setTargetRotations(m_targetRotations);
    // }

  }

  public void setSpeed(double speed) {
    m_motorLeft.set(speed);
  }

  public void setAxisSpeed(double speed) {
    m_targetRotations = Units.Rotations.of(Double.NaN);
    speed *= ELEVATOR.AXIS_MAX_SPEED;
    m_motorLeft.set(speed);
  }

  public void setZero() {
    m_encoder.setPosition(0);
  }

  public void stop() {
    m_motorLeft.stopMotor();
  }

  public AngularVelocity getVelocity() {
    return Units.RPM.of(m_encoder.getVelocity());
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
      ControlType.kMAXMotionPositionControl,
      ClosedLoopSlot.kSlot0,
      Math.copySign(arbFF, direction()),
      ArbFFUnits.kVoltage
    );
  }

  private int direction() {
    Angle diff = m_targetRotations.minus(getRotations());
    return (int) Math.copySign(1, diff.in(Units.Rotations));
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

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("elevator");

    builder.addBooleanProperty(
      "Save Config",
      () -> false,
      value -> {
        Preferences.setDouble("elevatorP", P);
        Preferences.setDouble("elevatorI", I);
        Preferences.setDouble("elevatorD", D);
        Preferences.setDouble("elevatorFF", arbFF);
        Preferences.setDouble("elevatorMaxVel", maxVel);
        Preferences.setDouble("elevatorMaxAcc", maxAcc);
      }
    );
  }
}

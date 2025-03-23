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
import frc.robot.Constants.LATERATOR;

// TODO: Goal: Get to scoring position at or before laterator can extend
public class LateratorTuningSubsystem implements Sendable {

  private SparkMax m_motor;
  private SparkClosedLoopController m_PIDController;
  private RelativeEncoder m_encoder;
  private Angle m_targetRotations = Units.Rotations.of(Double.NaN);

  private double P = 0.00008;
  private double I = 0;
  private double D = 0;
  private double arbFF = 0;
  private double velFF = 0.00025;
  private double maxVel = 2000; // Desired: 4600, Max: 5600
  private double maxAcc = 4800; // Desired: Unknown

  private SparkBaseConfig m_motorconfig = Constants.LATERATOR.MOTOR_CONFIG;

  public LateratorTuningSubsystem() {
    m_motor = new SparkMax(
      Constants.CAN_ID.LATERATOR_MOTOR,
      MotorType.kBrushless
    );
    m_motor.configure(
      m_motorconfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );
    m_encoder = m_motor.getEncoder();

    m_PIDController = m_motor.getClosedLoopController();

    Preferences.initDouble("lateratorP", P);
    Preferences.initDouble("lateratorI", I);
    Preferences.initDouble("lateratorD", D);
    Preferences.initDouble("lateratorFF", arbFF);
    Preferences.initDouble("lateratorVelFF", velFF);
    Preferences.initDouble("lateratorMaxVel", maxVel);
    Preferences.initDouble("lateratorMaxAcc", maxAcc);

    P = Preferences.getDouble("lateratorP", P);
    I = Preferences.getDouble("lateratorI", I);
    D = Preferences.getDouble("lateratorD", D);
    arbFF = Preferences.getDouble("lateratorFF", arbFF);
    velFF = Preferences.getDouble("lateratorVelFF", velFF);
    maxVel = Preferences.getDouble("lateratorMaxVel", maxVel);
    maxAcc = Preferences.getDouble("lateratorMaxAcc", maxAcc);

    displayDashboard();
    updatePIDs();
  }

  public void displayDashboard() {
    SmartDashboard.putNumber("Laterator P", P);
    SmartDashboard.putNumber("Laterator I", I);
    SmartDashboard.putNumber("Laterator D", D);
    SmartDashboard.putNumber("Laterator arbFF", arbFF);
    SmartDashboard.putNumber("Laterator velFF", velFF);
    SmartDashboard.putNumber("Laterator MaxVel", maxVel);
    SmartDashboard.putNumber("Laterator MaxAcc", maxAcc);
    SmartDashboard.putNumber("Motor Rotations", m_encoder.getPosition());
    SmartDashboard.putNumber("Motor Velocity", m_encoder.getVelocity());
    SmartDashboard.putBoolean("Is At Target", isAtTargetRotations());
    SmartDashboard.putNumber(
      "Calculated Distance",
      getDistance().in(Units.Inches)
    );
    SmartDashboard.putNumber("Laterator Setpoint", 0);
    SmartDashboard.putData("Save Laterator Config", this);
  }

  public void updatePIDs() {
    m_motorconfig.closedLoop.pidf(P, I, D, 0).velocityFF(velFF);

    m_motorconfig.closedLoop.smartMotion
      .maxAcceleration(maxAcc)
      .maxVelocity(maxVel);

    m_motor.configure(
      m_motorconfig,
      ResetMode.kNoResetSafeParameters,
      PersistMode.kNoPersistParameters
    );
  }

  public void updateDashboard() {
    double newP = SmartDashboard.getNumber("Laterator P", P);
    double newI = SmartDashboard.getNumber("Laterator I", I);
    double newD = SmartDashboard.getNumber("Laterator D", D);
    double newFF = SmartDashboard.getNumber("Laterator arbFF", arbFF);
    double newVelFF = SmartDashboard.getNumber("Laterator velFF", velFF);
    double newMaxVel = SmartDashboard.getNumber("Laterator MaxVel", maxVel);
    double newMaxAcc = SmartDashboard.getNumber("Laterator MaxAcc", maxAcc);
    SmartDashboard.putNumber("Motor Rotations", m_encoder.getPosition());
    SmartDashboard.putNumber("Motor Velocity", m_encoder.getVelocity());
    m_targetRotations = Rotations.of(
      SmartDashboard.getNumber("Laterator Setpoint", 0)
    );
    SmartDashboard.putBoolean("Is At Target", isAtTargetRotations());
    SmartDashboard.putNumber(
      "Calculated Distance",
      getDistance().in(Units.Inches)
    );

    if (
      newP != P ||
      newI != I ||
      newD != D ||
      newFF != arbFF ||
      newVelFF != velFF ||
      newMaxVel != maxVel ||
      newMaxAcc != maxAcc
    ) {
      P = newP;
      I = newI;
      D = newD;
      arbFF = newFF;
      velFF = newVelFF;
      maxVel = newMaxVel;
      maxAcc = newMaxAcc;
      updatePIDs();
    }
  }

  public void teleopPeriodic() {
    // if (SmartDashboard.getBoolean("UseDistance", false)) {

    // double distanceSetpoint;
    // distanceSetpoint = SmartDashboard.getNumber("Laterator Distance Setpoint", 0);
    // setTargetDistance(Units.Feet.of(distanceSetpoint));
    // } else {

    setTargetRotations(m_targetRotations);
    // }

  }

  public void setSpeed(double speed) {
    m_motor.set(speed);
  }

  public void setAxisSpeed(double speed) {
    // m_targetRotations = Units.Rotations.of(Double.NaN);
    speed *= LATERATOR.AXIS_MAX_SPEED;
    m_motor.set(speed);
  }

  public void setZero() {
    m_encoder.setPosition(0);
  }

  public void stop() {
    m_motor.stopMotor();
  }

  public AngularVelocity getVelocity() {
    return Units.RPM.of(m_encoder.getVelocity());
  }

  private Angle getRotations() {
    return Units.Rotations.of(m_encoder.getPosition());
  }

  public Distance getDistance() {
    return (
      LATERATOR.OUTPUT_PULLEY_CIRCUMFERENCE.times(
        getRotations().div(LATERATOR.GEAR_RATIO).in(Units.Rotations)
      )
    );
  }

  public void setTargetRotations(Angle targetRotations) {
    m_targetRotations = targetRotations;
    m_PIDController.setReference(
      m_targetRotations.in(Units.Rotations),
      ControlType.kSmartMotion,
      ClosedLoopSlot.kSlot0,
      arbFF,
      ArbFFUnits.kVoltage
    );
  }

  public void setTargetDistance(Distance targetDistance) {
    Angle rotations = Units.Rotations.of(
      targetDistance
        .div(LATERATOR.OUTPUT_PULLEY_CIRCUMFERENCE)
        .times(LATERATOR.GEAR_RATIO)
        .magnitude()
    );
    System.out.println("ROTATIONS: " + rotations.in(Rotations));
    setTargetRotations(rotations);
  }

  private boolean isAtTargetRotations() {
    return m_targetRotations.isNear(
      getRotations(),
      LATERATOR.MAX_MOTION_ALLOWED_ERROR_PERCENT
    );
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("laterator");

    builder.addBooleanProperty(
      "Save Config",
      () -> false,
      value -> {
        Preferences.setDouble("lateratorP", P);
        Preferences.setDouble("lateratorI", I);
        Preferences.setDouble("lateratorD", D);
        Preferences.setDouble("lateratorFF", arbFF);
        Preferences.setDouble("lateratorVelFF", velFF);
        Preferences.setDouble("lateratorMaxVel", maxVel);
        Preferences.setDouble("lateratorMaxAcc", maxAcc);
      }
    );
  }
}

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
import edu.wpi.first.units.measure.MomentOfInertia;
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

  private double P = 0.0;
  private double I = 0;
  private double D = 0;
  private double arbFF = 0.15;
  private double velFF = 0.0003;
  private double maxVel = 4600; // Desired: 4600, Max: 5600
  private double maxAcc = 10000; // Desired: 18400

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

    Preferences.initDouble("elevatorP", P);
    Preferences.initDouble("elevatorI", I);
    Preferences.initDouble("elevatorD", D);
    Preferences.initDouble("elevatorFF", arbFF);
    Preferences.initDouble("elevatorVelFF", velFF);
    Preferences.initDouble("elevatorMaxVel", maxVel);
    Preferences.initDouble("elevatorMaxAcc", maxAcc);

    P = Preferences.getDouble("elevatorP", P);
    I = Preferences.getDouble("elevatorI", I);
    D = Preferences.getDouble("elevatorD", D);
    arbFF = Preferences.getDouble("elevatorFF", arbFF);
    velFF = Preferences.getDouble("elevatorVelFF", velFF);
    maxVel = Preferences.getDouble("elevatorMaxVel", maxVel);
    maxAcc = Preferences.getDouble("elevatorMaxAcc", maxAcc);

    displayDashboard();
    updatePIDs();
  }

  public void displayDashboard() {
    SmartDashboard.putNumber("Elevator P", P);
    SmartDashboard.putNumber("Elevator I", I);
    SmartDashboard.putNumber("Elevator D", D);
    SmartDashboard.putNumber("Elevator arbFF", arbFF);
    SmartDashboard.putNumber("Elevator velFF", velFF);
    SmartDashboard.putNumber("Elevator MaxVel", maxVel);
    SmartDashboard.putNumber("Elevator MaxAcc", maxAcc);
    SmartDashboard.putNumber("Motor Rotations", m_encoder.getPosition());
    SmartDashboard.putNumber("Motor Velocity", m_encoder.getVelocity());
    SmartDashboard.putBoolean("Is At Target", isAtTargetRotations());
    SmartDashboard.putNumber(
      "Calculated Distance",
      getDistance().in(Units.Inches)
    );
    SmartDashboard.putNumber("Elevator Setpoint", 0);
    SmartDashboard.putData("Save Elevator Config", this);
  }

  public void updatePIDs() {
    // Rev recommends not using velocity feed forward for max motion positional control
    m_motorconfig.closedLoop.pidf(P, I, D, velFF);

    m_motorconfig.closedLoop.smartMotion
      .maxAcceleration(maxAcc)
      .maxVelocity(maxVel)
      .allowedClosedLoopError(0.3);

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
    double newVelFF = SmartDashboard.getNumber("Elevator velFF", velFF);
    double newMaxVel = SmartDashboard.getNumber("Elevator MaxVel", maxVel);
    double newMaxAcc = SmartDashboard.getNumber("Elevator MaxAcc", maxAcc);
    SmartDashboard.putNumber("Motor Rotations", m_encoder.getPosition());
    SmartDashboard.putNumber("Motor Velocity", m_encoder.getVelocity());
    m_targetRotations = Rotations.of(
      SmartDashboard.getNumber("Elevator Setpoint", 0)
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
      ELEVATOR.OUTPUT_PULLEY_CIRCUMFERENCE.times(
        getRotations().div(ELEVATOR.GEAR_RATIO).in(Units.Rotations)
      )
    );
  }

  private void setTargetRotations(Angle targetRotations) {
    System.out.println(
      "Setting rotations: " + targetRotations.in(Units.Rotation)
    );
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
        .div(ELEVATOR.OUTPUT_PULLEY_CIRCUMFERENCE)
        .times(ELEVATOR.GEAR_RATIO)
        .magnitude()
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
        Preferences.setDouble("elevatorVelFF", velFF);
        Preferences.setDouble("elevatorMaxVel", maxVel);
        Preferences.setDouble("elevatorMaxAcc", maxAcc);
      }
    );
  }
}

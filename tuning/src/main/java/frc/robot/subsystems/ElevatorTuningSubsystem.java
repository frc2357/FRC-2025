package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
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

public class ElevatorTuningSubsystem implements Sendable {

  private SparkMax m_motorLeft;
  private SparkMax m_motorRight;
  private SparkClosedLoopController m_PIDController;
  private RelativeEncoder m_encoder;
  private Angle m_targetRotations = Units.Rotations.of(Double.NaN);

  private double P = 0;
  private double I = 0;
  private double D = 0;
  private double FF = 0;
  private double maxVel = 0;
  private double maxAcc = 0;

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

    displayDashboard();
  }

  public void updatePIDs() {
    m_motorconfig.closedLoop.pidf(P, I, D, FF);

    m_motorconfig.closedLoop.maxMotion
      .maxAcceleration(maxAcc)
      .maxVelocity(maxVel);

    m_motorLeft.configure(
      m_motorconfig,
      ResetMode.kNoResetSafeParameters,
      PersistMode.kNoPersistParameters
    );
  }

  public void displayDashboard() {
    SmartDashboard.putNumber("Elevator P", P);
    SmartDashboard.putNumber("Elevator I", I);
    SmartDashboard.putNumber("Elevator D", D);
    SmartDashboard.putNumber("Elevator FF", FF);
    SmartDashboard.putNumber("Elevator MaxVel", maxVel);
    SmartDashboard.putNumber("Elevator MaxAcc", maxAcc);
    SmartDashboard.putNumber("Motor Rotations", m_encoder.getPosition());
    SmartDashboard.putBoolean("Is At Target", isAtTargetRotations());
    SmartDashboard.putNumber("Calculated Distance", getDistance().magnitude());
    SmartDashboard.putNumber("Elevator Setpoint", 0);
    SmartDashboard.putData("Save PID", this);
  }

  public void updateDashboard() {
    double newP = SmartDashboard.getNumber("Elevator P", P);
    double newI = SmartDashboard.getNumber("Elevator I", I);
    double newD = SmartDashboard.getNumber("Elevator D", D);
    double newFF = SmartDashboard.getNumber("Elevator FF", FF);
    double newMaxVel = SmartDashboard.getNumber("Elevator MaxVel", maxVel);
    double newMaxAcc = SmartDashboard.getNumber("Elevator MaxAcc", maxAcc);
    SmartDashboard.putNumber("Motor Rotations", m_encoder.getPosition());
    m_targetRotations = Rotations.of(
      SmartDashboard.getNumber("Elevator Setpoint", 0)
    );
    SmartDashboard.putBoolean("Is At Target", isAtTargetRotations());
    SmartDashboard.putNumber("Calculated Distance", getDistance().magnitude());

    if (
      newP != P ||
      newI != I ||
      newD != D ||
      newFF != FF ||
      newMaxVel != maxVel ||
      newMaxAcc != maxAcc
    ) {
      P = newP;
      I = newI;
      D = newD;
      FF = newFF;
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

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("save");

    builder.addBooleanProperty(
      "Save Preferences",
      () -> false,
      value -> {
        Preferences.initDouble("elevatorP", P);
        Preferences.initDouble("elevatorI", I);
        Preferences.initDouble("elevatorD", D);
        Preferences.initDouble("elevatorFF", FF);
        Preferences.initDouble("elevatorMaxVel", maxAcc);
        Preferences.initDouble("elevatorMaxAcc", maxVel);
      }
    );
  }
}

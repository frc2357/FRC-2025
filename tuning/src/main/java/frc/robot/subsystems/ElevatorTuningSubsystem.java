package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
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
  private RelativeEncoder m_encoder;
  private Distance m_targetDistance = Units.Meters.of(Double.NaN);

  private double P = 0.016;
  private double I = 0;
  private double D = 0;
  private double kG = 0.013;
  private double kV = 0.0;
  private double kS = 0.0;
  private double kA = 0.0;
  private double maxVel = 0.5; // Desired: 1.977, Max: 2.4
  private double maxAcc = 0.5; // Desired: 4

  private SparkBaseConfig m_motorconfig = Constants.ELEVATOR.MOTOR_CONFIG_LEFT;

  private ProfiledPIDController m_PIDController = new ProfiledPIDController(
    P,
    I,
    D,
    new TrapezoidProfile.Constraints(maxVel, maxAcc)
  );

  private ElevatorFeedforward m_feedforward = new ElevatorFeedforward(
    kS,
    kG,
    kV,
    kA
  );

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

    Preferences.initDouble("elevatorP", 0);
    Preferences.initDouble("elevatorI", 0);
    Preferences.initDouble("elevatorD", 0);
    Preferences.initDouble("elevatorKG", 0);
    Preferences.initDouble("elevatorKA", 0);
    Preferences.initDouble("elevatorKS", 0);
    Preferences.initDouble("elevatorKV", 0);
    Preferences.initDouble("elevatorMaxVel", 0);
    Preferences.initDouble("elevatorMaxAcc", 0);

    P = Preferences.getDouble("elevatorP", P);
    I = Preferences.getDouble("elevatorI", I);
    D = Preferences.getDouble("elevatorD", D);
    kS = Preferences.getDouble("elevatorKG", kG);
    kG = Preferences.getDouble("elevatorKA", kA);
    kV = Preferences.getDouble("elevatorKS", kS);
    kA = Preferences.getDouble("elevatorKV", kV);
    maxVel = Preferences.getDouble("elevatorMaxVel", maxVel);
    maxAcc = Preferences.getDouble("elevatorMaxAcc", maxAcc);

    displayDashboard();
    updatePIDs();
  }

  public void displayDashboard() {
    SmartDashboard.putNumber("Elevator P", P);
    SmartDashboard.putNumber("Elevator I", I);
    SmartDashboard.putNumber("Elevator D", D);
    SmartDashboard.putNumber("Elevator kG", kG);
    SmartDashboard.putNumber("Elevator kA", kA);
    SmartDashboard.putNumber("Elevator kS", kS);
    SmartDashboard.putNumber("Elevator kV", kV);
    SmartDashboard.putNumber("Elevator MaxVel", maxVel);
    SmartDashboard.putNumber("Elevator MaxAcc", maxAcc);
    SmartDashboard.putNumber("Motor Rotations", m_encoder.getPosition());
    SmartDashboard.putNumber("Motor Velocity", m_encoder.getVelocity());
    SmartDashboard.putBoolean("Is At Target", isAtTargetDistance());
    SmartDashboard.putNumber(
      "Calculated Distance",
      getDistance().in(Units.Inches)
    );
    SmartDashboard.putNumber("Elevator Setpoint", 0);
    SmartDashboard.putData("Save Elevator Config", this);
  }

  public void updatePIDs() {
    // Rev recommends not using velocity feed forward for max motion positional control
    m_PIDController.setPID(P, I, D);
    m_PIDController.setConstraints(
      new TrapezoidProfile.Constraints(maxVel, maxAcc)
    );
    m_feedforward.setKa(kA);
    m_feedforward.setKg(kG);
    m_feedforward.setKs(kS);
    m_feedforward.setKv(kV);
  }

  public void updateDashboard() {
    double newP = SmartDashboard.getNumber("Elevator P", P);
    double newI = SmartDashboard.getNumber("Elevator I", I);
    double newD = SmartDashboard.getNumber("Elevator D", D);
    double newkG = SmartDashboard.getNumber("Elevator kG", kG);
    double newkA = SmartDashboard.getNumber("Elevator kA", kA);
    double newkS = SmartDashboard.getNumber("Elevator kS", kS);
    double newkV = SmartDashboard.getNumber("Elevator kV", kV);
    double newMaxVel = SmartDashboard.getNumber("Elevator MaxVel", maxVel);
    double newMaxAcc = SmartDashboard.getNumber("Elevator MaxAcc", maxAcc);
    SmartDashboard.putNumber("Motor Rotations", m_encoder.getPosition());
    SmartDashboard.putNumber("Motor Velocity", m_encoder.getVelocity());
    m_targetDistance = Units.Meters.of(
      SmartDashboard.getNumber("Elevator Setpoint", 0)
    );
    SmartDashboard.putBoolean("Is At Target", isAtTargetDistance());
    SmartDashboard.putNumber(
      "Calculated Distance",
      getDistance().in(Units.Inches)
    );

    if (
      newP != P ||
      newI != I ||
      newD != D ||
      newkG != kG ||
      newkA != kA ||
      newkS != kS ||
      newkV != kV ||
      newMaxVel != maxVel ||
      newMaxAcc != maxAcc
    ) {
      P = newP;
      I = newI;
      D = newD;
      kG = newkG;
      kA = newkA;
      kS = newkS;
      kV = newkV;
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

    reachDistance(m_targetDistance);
    // }

  }

  public void setSpeed(double speed) {
    m_motorLeft.set(speed);
  }

  public void setAxisSpeed(double speed) {
    m_targetDistance = Units.Meters.of(Double.NaN);
    speed *= ELEVATOR.AXIS_MAX_SPEED;
    m_motorLeft.set(speed);
  }

  public void setZero() {
    m_encoder.setPosition(0);
  }

  public void stop() {
    m_motorLeft.stopMotor();
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

  public void reachDistance(Distance targetDistance) {
    m_targetDistance = targetDistance;
    double volts = MathUtil.clamp(
      m_PIDController.calculate(
        getDistance().in(Units.Meters),
        m_targetDistance.in(Units.Meters)
      ) +
      m_feedforward.calculateWithVelocities(
        getVelocity().in(Units.MetersPerSecond),
        m_PIDController.getSetpoint().velocity
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
    return Units.MetersPerSecond.of(vel);
  }

  private boolean isAtTargetDistance() {
    return m_targetDistance.isNear(
      getRotations(),
      ELEVATOR.ALLOWED_POSITION_ERROR_PERCENT
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
        Preferences.initDouble("elevatorKG", kG);
        Preferences.initDouble("elevatorKA", kA);
        Preferences.initDouble("elevatorKS", kS);
        Preferences.initDouble("elevatorKV", kV);
        Preferences.setDouble("elevatorMaxVel", maxVel);
        Preferences.setDouble("elevatorMaxAcc", maxAcc);
      }
    );
  }
}

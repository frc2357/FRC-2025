package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;

import java.util.Spliterators.AbstractLongSpliterator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CAN_ID;
import frc.robot.Constants.ELEVATOR;

//import edu.wpi.first.math.controller.PIDController;

public class ElevatorTuningSubsystem extends SubsystemBase {

    private SparkMax m_motorLeft;
    private SparkMax m_motorRight;
    private SparkClosedLoopController m_PIDController;
    private RelativeEncoder m_encoder;
    private Angle m_targetRotations = Units.Rotations.of(Double.NaN);

    private double kP = 0;
    private double kI = 0;
    private double kD = 0;
    private double kFF = 0;
    private double maxVel = 0;
    private double maxAcc = 0;

    private double axisMaxSpeed = 0;

    private SparkBaseConfig motorconfig = Constants.ELEVATOR.MOTOR_CONFIG_LEFT;

    // private PIDController m_PidController

    public ElevatorTuningSubsystem() {
        m_motorLeft = new SparkMax(
                Constants.CAN_ID.ELEVATOR_LEFT_MOTOR,
                MotorType.kBrushless);
        m_motorRight = new SparkMax(
                Constants.CAN_ID.ELEVATOR_RIGHT_MOTOR,
                MotorType.kBrushless);

        updatePIDs();

        m_motorRight.configure(
                Constants.ELEVATOR.MOTOR_CONFIG_RIGHT,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        m_motorLeft.configure(
                motorconfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        m_PIDController = m_motorLeft.getClosedLoopController();
        displayDashboard();
    }

    public void updatePIDs() {
        motorconfig.closedLoop.pidf(
                kP,
                kI,
                axisMaxSpeed,
                kFF);

        motorconfig.closedLoop.maxMotion
                .maxAcceleration(maxAcc)
                .maxVelocity(maxVel);
    }

    public void displayDashboard() {
        SmartDashboard.putNumber("Elevator P", kP);
        SmartDashboard.putNumber("Elevator I", kI);
        SmartDashboard.putNumber("Elevator D", kD);
        SmartDashboard.putNumber("Elevator FF", kFF);
        SmartDashboard.putNumber("Elevator MaxVel", maxVel);
        SmartDashboard.putNumber("Elevator MaxAcc", maxAcc);
        SmartDashboard.putNumber("Elevator Setpoint", 0);
    }

    public void updateDashboard() {
        kP = SmartDashboard.getNumber("Arm P", kP);
        kI = SmartDashboard.getNumber("Arm I", kI);
        kD = SmartDashboard.getNumber("Arm D", kD);
        kFF = SmartDashboard.getNumber("Arm FF", kFF);
        maxVel = SmartDashboard.getNumber("Arm MaxVel", maxVel);
        maxAcc = SmartDashboard.getNumber("Arm MaxAcc", maxAcc);

        SmartDashboard.putNumber("Motor Rotations", m_encoder.getPosition());
        SmartDashboard.putBoolean("Is At Setpoint", isAtSetpoint());

        updatePIDs();

    }

    public void teleopPeriodic() {

        // if (SmartDashboard.getBoolean("UseDistance", false)) {

        // double distanceSetpoint;
        // distanceSetpoint = SmartDashboard.getNumber("Elevator Distance Setpoint", 0);
        // setTargetDistance(Units.Feet.of(distanceSetpoint));
        // } else {

        double rotationSetpoint;
        rotationSetpoint = SmartDashboard.getNumber("Elevator Rotation Setpoint", 0);
        setTargetRotations(Degrees.of(rotationSetpoint));
        // }

    }

    public void setSpeed(double speed) {
        m_motorLeft.set(speed);
        setTargetRotations(Units.Rotations.of(Double.NaN));
    }

    public void stop() {
        m_motorLeft.stopMotor();
        setTargetRotations(Units.Rotations.of(Double.NaN));
    }

    public AngularVelocity getVelocity() {
        return Units.RotationsPerSecond.of(m_encoder.getVelocity() / 60);
    }

    public void setZero() {
        m_encoder.setPosition(0);
    }

    public Angle getRotations() {

        return Units.Rotations.of(m_encoder.getPosition());
    }

    public void setTargetRotations(Angle targetRotations) {
        m_targetRotations = targetRotations;
        m_PIDController.setReference(
                m_targetRotations.in(Units.Rotations),
                ControlType.kMAXMotionPositionControl);
    }

    public void setTargetDistance(Distance targetDistance) {
        Angle rotations = Units.Rotations.of(
                targetDistance.div(ELEVATOR.MOTOR_PULLEY_PITCH_DIAMETER).magnitude());
        setTargetRotations(rotations);
    }

    public boolean isAtTargetRotations() {
        return m_targetRotations.isNear(
                getRotations(),
                ELEVATOR.MAX_MOTION_ALLOWED_ERROR_PERCENT);
    }

    public boolean isAtSetpoint() {
        return isAtTargetRotations();
    }

    public void setAxisSpeed(double speed) {
        m_targetRotations = Units.Rotations.of(Double.NaN);
        speed *= ELEVATOR.AXIS_MAX_SPEED;
        m_motorLeft.set(speed);
    }
}
package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//import edu.wpi.first.math.controller.PIDController;
import frc.robot.util.Utility;

public class Elevator extends SubsystemBase {
    private SparkMax m_motorLeft;
    private SparkMax m_motorRight;
    SparkClosedLoopController m_PIDControllerLeft;
    SparkClosedLoopController m_PIDControllerRight;
    RelativeEncoder m_encoderLeft;
    RelativeEncoder m_encoderRight;
    double m_targetRotations;

    private double kP = 0.001;
    private double kI = 0;
    private double kD = 0;
    private double kFF = 0;
    private double maxVel = 6000;// 5700; // These are the new rpms, not rev throughbore encoder rpms
    private double maxAcc = 24000; // 5700;

    SparkBaseConfig m_motorRightConfig = Constants.ELEVATOR.MOTOR_CONFIG_RIGHT;

    SparkBaseConfig m_motorLeftConfig = Constants.ELEVATOR.MOTOR_CONFIG_LEFT;

    double maxError;

    // private PIDController m_PidController

    public Elevator() {
        m_motorLeft = new SparkMax(Constants.CAN_ID.ELEVATOR_LEFT_MOTOR, MotorType.kBrushless);
        m_motorRight = new SparkMax(Constants.CAN_ID.ELEVATOR_RIGHT_MOTOR, MotorType.kBrushless);

        m_PIDControllerLeft = m_motorLeft.getClosedLoopController();
        m_PIDControllerRight = m_motorLeft.getClosedLoopController();

        m_encoderLeft = m_motorLeft.getEncoder();
        m_encoderRight = m_motorRight.getEncoder();

        config();
        display();
    }

    public void config() {

        m_motorRightConfig.closedLoop.pidf(
                kP,
                kI,
                kD,
                kFF)
                .outputRange(-1, 1);

        m_motorLeftConfig.closedLoop.pidf(
                kP,
                kI,
                kD,
                kFF)
                .outputRange(-1, 1);

        m_motorRightConfig.closedLoop.maxMotion
                .allowedClosedLoopError(maxError)
                .maxAcceleration(maxAcc)
                .maxVelocity(maxVel);

        m_motorLeftConfig.closedLoop.maxMotion
                .allowedClosedLoopError(maxError)
                .maxAcceleration(maxAcc)
                .maxVelocity(maxVel);

        m_motorRight.configure(m_motorRightConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        m_motorLeft.configure(m_motorLeftConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    public void display() {
        SmartDashboard.putNumber("Arm P", kP);
        SmartDashboard.putNumber("Arm I", kI);
        SmartDashboard.putNumber("Arm D", kD);
        SmartDashboard.putNumber("Arm FF", kFF);
        SmartDashboard.putNumber("Arm MaxVel", maxVel);
        SmartDashboard.putNumber("Arm MaxAcc", maxAcc);
        // This is the REV Throughbore position setpoint
        SmartDashboard.putNumber("Arm Setpoint", 0);
        SmartDashboard.putNumber("Arm maxError", maxError);
    }

    public void update() {
        kP = SmartDashboard.getNumber("Arm P", kP);
        kI = SmartDashboard.getNumber("Arm I", kI);
        kD = SmartDashboard.getNumber("Arm D", kD);
        kFF = SmartDashboard.getNumber("Arm FF", kFF);
        maxVel = SmartDashboard.getNumber("Arm MaxVel", maxVel);
        maxAcc = SmartDashboard.getNumber("Arm MaxAcc", maxAcc);
        maxError = SmartDashboard.getNumber("Arm maxError", maxError);

        config();

        SmartDashboard.putNumber("Arm RPMs", m_encoderRight.getVelocity());

        SmartDashboard.putNumber("Arm Pos", m_encoderRight.getPosition());
    }

    public void setSpeed(double speed) {
        // m_motorLeft.set(speed);
        m_motorRight.set(speed);
        setTargetRotations(Double.NaN);

    }

    public void teleopPeriodic() {
        double rotationSetpoint;
        rotationSetpoint = SmartDashboard.getNumber("Arm Setpoint", 0);
        setTargetRotations(rotationSetpoint);
    }

    public void stop() {
        // m_motorLeft.stopMotor();
        m_motorRight.stopMotor();
        setTargetRotations(Double.NaN);
    }

    public double getVelocity() {
        return m_encoderRight.getVelocity();
    }

    public void setZero() {
        m_encoderRight.setPosition(0);
        // m_encoderLeft.setPosition(0);
    }

    public double getRotations() {
        return m_encoderRight.getPosition();
    }

    public void setTargetRotations(double targetRotations) {
        m_targetRotations = targetRotations;
        m_PIDControllerRight.setReference(m_targetRotations, ControlType.kMAXMotionPositionControl);
        // m_PIDControllerLeft.setReference(m_targetRotations,
        // ControlType.kMAXMotionPositionControl);
    }

    public boolean isAtTargetRotations() {
        return Utility.isWithinTolerance(getRotations(), m_targetRotations,
                maxError);
    };

}

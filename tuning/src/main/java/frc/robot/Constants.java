package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.BooleanSupplier;

import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.auto.AutoFactory.AutoBindings;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class Constants {

  public static final class CAN_ID {
    public static final int ELEVATOR_LEFT_MOTOR = -1;
    public static final int ELEVATOR_RIGHT_MOTOR = -1;
  }

  public static final class ELEVATOR {

    public static final double MAX_MOTION_ALLOWED_ERROR = 0;

    public static final SparkBaseConfig MOTOR_CONFIG_LEFT = new SparkMaxConfig()
        .idleMode(IdleMode.kBrake)
        .follow(CAN_ID.ELEVATOR_RIGHT_MOTOR, true);

    public static final SparkBaseConfig MOTOR_CONFIG_RIGHT = new SparkMaxConfig()
        .idleMode(IdleMode.kBrake);

    public static final ClosedLoopConfig CLOSED_LOOP_CONFIG_RIGHT = MOTOR_CONFIG_RIGHT.closedLoop
        .pidf(0, 0, 0, 0)
        .outputRange(-1, 1);

    public static final ClosedLoopConfig CLOSED_LOOP_CONFIG_LEFT = MOTOR_CONFIG_LEFT.closedLoop
        .pidf(0, 0, 0, 0)
        .outputRange(-1, 1);

    public static final MAXMotionConfig MAX_MOTION_CONFIG_RIGHT = CLOSED_LOOP_CONFIG_RIGHT.maxMotion
        .allowedClosedLoopError(MAX_MOTION_ALLOWED_ERROR)
        .maxAcceleration(0)
        .maxVelocity(0);

    public static final MAXMotionConfig MAX_MOTION_CONFIG_LEFT = CLOSED_LOOP_CONFIG_LEFT.maxMotion
        .allowedClosedLoopError(MAX_MOTION_ALLOWED_ERROR)
        .maxAcceleration(0)
        .maxVelocity(0);

  }

  public static final class SWERVE {
    public static final AngularVelocity MAX_ANGULAR_RATE = AngularVelocity.ofBaseUnits(Math.PI * 2, RotationsPerSecond);

    public static final Distance STATIC_FEEDFORWARD = Distance.ofBaseUnits(0.094545, Meters);
    public static final Time TIME_TO_COAST = Time.ofBaseUnits(5, Seconds);
  }

  public static final class CONTROLLER {
    public static final int DRIVE_CONTROLLER_PORT = 0;
    public static final double DRIVE_CONTROLLER_DEADBAND = 0.01;
    public static final double SWERVE_TRANSLATIONAL_DEADBAND = 0.01;
    public static final double SWERVE_ROTATIONAL_DEADBAND = 0.01;
    public static final double DRIVE_RUMBLE_INTENSITY = .5;
    public static final double DRIVE_RUMBLE_SECONDS = 2;

    public static final double DRIVE_TRANSLATE_INTAKE_THRESHOLD = 0.9;
  }

  public static final class CHOREO {
    public static final PIDController X_CONTROLLER = new PIDController(5, 0, 0);
    public static final PIDController Y_CONTROLLER = new PIDController(5, 0, 0);
    public static final PIDController ROTATION_CONTROLLER = new PIDController(1, 0, 0);

    /**
     * returns true if the alliance is red, false if it is blue
     */
    public static final BooleanSupplier CHOREO_AUTO_MIRROR_PATHS = new BooleanSupplier() {
      @Override
      public boolean getAsBoolean() {
        return Robot.state.getAlliance() == Alliance.Red;
      }
    };

    public static final AutoBindings AUTO_BINDINGS = new AutoBindings();
    public static final AutoFactory AUTO_FACTORY = new AutoFactory(
        Robot.swerve::getPose2d,
        Robot.swerve::setPose2d,
        Robot.swerve::followChoreoPath,
        true,
        Robot.swerve,
        AUTO_BINDINGS);
  }
}

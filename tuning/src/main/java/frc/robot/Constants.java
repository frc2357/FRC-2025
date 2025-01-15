package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import choreo.Choreo;
import choreo.auto.AutoFactory;
//import choreo.auto.AutoFactory.AutoBindings;
import choreo.trajectory.SwerveSample;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.CAN_ID;
import java.util.function.BooleanSupplier;

public final class Constants {

  public static final class CAN_ID {

    public static final int ELEVATOR_LEFT_MOTOR = -1;
    public static final int ELEVATOR_RIGHT_MOTOR = -1;
  }

  public static final class SWERVE {

    public static final AngularVelocity MAX_ANGULAR_RATE =
      AngularVelocity.ofBaseUnits(Math.PI * 2, RotationsPerSecond);

    public static final Distance STATIC_FEEDFORWARD = Distance.ofBaseUnits(
      0.094545,
      Meters
    );
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
    public static final PIDController ROTATION_CONTROLLER = new PIDController(
      1,
      0,
      0
    );
    /**
     * returns true if the alliance is red, false if it is blue
     */
    /*
     * public static final BooleanSupplier CHOREO_AUTO_MIRROR_PATHS = new
     * BooleanSupplier() {
     *
     * @Override
     * public boolean getAsBoolean() {
     * return Robot.state.getAlliance() == Alliance.Red;
     * }
     *
     * };
     * /*
     * public static final AutoBindings AUTO_BINDINGS = new AutoBindings();
     * public static final AutoFactory AUTO_FACTORY = new AutoFactory(
     * Robot.swerve::getPose2d,
     * Robot.swerve::setPose2d,
     * Robot.swerve::followChoreoPath,
     * true,
     * Robot.swerve,
     * AUTO_BINDINGS);
     */

  }

  public static final class ELEVATOR {

    public static final SparkBaseConfig MOTOR_CONFIG_LEFT = new SparkMaxConfig()
      .idleMode(IdleMode.kBrake)
      .inverted(false);

    public static final SparkBaseConfig MOTOR_CONFIG_RIGHT =
      new SparkMaxConfig()
        .idleMode(IdleMode.kBrake)
        .follow(CAN_ID.ELEVATOR_LEFT_MOTOR, true);

    public static final double LEFT_MOTOR_CLOSED_LOOP_P = 0;
    public static final double LEFT_MOTOR_CLOSED_LOOP_I = 0;
    public static final double LEFT_MOTOR_CLOSED_LOOP_D = 0;
    public static final double LEFT_MOTOR_CLOSED_LOOP_F = 0;

    public static final ClosedLoopConfig CLOSED_LOOP_CONFIG_LEFT =
      MOTOR_CONFIG_LEFT.closedLoop
        .pidf(
          LEFT_MOTOR_CLOSED_LOOP_P,
          LEFT_MOTOR_CLOSED_LOOP_I,
          LEFT_MOTOR_CLOSED_LOOP_D,
          LEFT_MOTOR_CLOSED_LOOP_F
        )
        .outputRange(-1, 1);

    public static final double MAX_MOTION_ALLOWED_ERROR_PERCENT = 0.03;

    public static final MAXMotionConfig MAX_MOTION_CONFIG_LEFT =
      CLOSED_LOOP_CONFIG_LEFT.maxMotion
        .allowedClosedLoopError(MAX_MOTION_ALLOWED_ERROR_PERCENT)
        .maxAcceleration(0)
        .maxVelocity(0);

    public static final int ENCODER_COUNTS_PER_REV = 8196;

    public static final double GEAR_RATIO = 50 / 14;
    public static final Distance MOTOR_PULLEY_PITCH_DIAMETER = Units.Inches.of(
      2.256
    );

    public static final double AXIS_MAX_SPEED = 0.1;

    public static final Distance[] ELEVATOR_HEIGHT_SETPOINTS = {};
  }

  public static final class CUSTOM_UNITS {

    // These units are ONLY for the output shaft on the neo. Any pulley will require
    // the addition of a gear ratio.
    public static final Distance NEO_SHAFT_CIRCUMFERENCE = Units.Millimeters.of(
      8 * Math.PI
    );
    public static final AngleUnit NEO_ENCODER_TICK = Units.derive(
      Units.Revolutions
    )
      .splitInto(42)
      .named("Neo Encoder Tick")
      .symbol("NET")
      .make();
  }
}

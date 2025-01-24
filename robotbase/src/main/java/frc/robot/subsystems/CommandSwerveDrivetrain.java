package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import choreo.trajectory.SwerveSample;
import choreo.util.ChoreoAllianceFlipUtil;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.CHOREO;
import frc.robot.Robot;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import java.util.function.Supplier;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain
  extends TunerSwerveDrivetrain
  implements Subsystem {

  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private Notifier m_simNotifier = null;
  private double m_lastSimTime;

  /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
  private static final Rotation2d kBlueAlliancePerspectiveRotation =
    Rotation2d.kZero;
  /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
  private static final Rotation2d kRedAlliancePerspectiveRotation =
    Rotation2d.k180deg;
  /* Keep track if we've ever applied the operator perspective before or not */
  private boolean m_hasAppliedOperatorPerspective = false;

  /* Swerve requests to apply during SysId characterization */
  private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization =
    new SwerveRequest.SysIdSwerveTranslation();
  private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization =
    new SwerveRequest.SysIdSwerveSteerGains();
  private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization =
    new SwerveRequest.SysIdSwerveRotation();

  /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
  private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
    new SysIdRoutine.Config(
      null, // Use default ramp rate (1 V/s)
      Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
      null, // Use default timeout (10 s)
      // Log state with SignalLogger class
      state ->
        SignalLogger.writeString("SysIdTranslation_State", state.toString())
    ),
    new SysIdRoutine.Mechanism(
      output -> setControl(m_translationCharacterization.withVolts(output)),
      null,
      this
    )
  );

  /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
  private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
    new SysIdRoutine.Config(
      null, // Use default ramp rate (1 V/s)
      Volts.of(7), // Use dynamic voltage of 7 V
      null, // Use default timeout (10 s)
      // Log state with SignalLogger class
      state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
    ),
    new SysIdRoutine.Mechanism(
      volts -> setControl(m_steerCharacterization.withVolts(volts)),
      null,
      this
    )
  );

  /*
   * SysId routine for characterizing rotation.
   * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
   * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
   */
  private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
    new SysIdRoutine.Config(
      /* This is in radians per second², but SysId only supports "volts per second" */
      Volts.of(Math.PI / 6).per(Second),
      /* This is in radians per second, but SysId only supports "volts" */
      Volts.of(Math.PI),
      null, // Use default timeout (10 s)
      // Log state with SignalLogger class
      state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
    ),
    new SysIdRoutine.Mechanism(
      output -> {
        /* output is actually radians per second, but SysId only supports "volts" */
        setControl(
          m_rotationCharacterization.withRotationalRate(output.in(Volts))
        );
        /* also log the requested output for SysId */
        SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
      },
      null,
      this
    )
  );

  /* The SysId routine to test */
  private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

  private final SwerveRequest.RobotCentric m_robotRelative =
    new SwerveRequest.RobotCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors;
  private final SwerveRequest.FieldCentric m_fieldRelative =
    new SwerveRequest.FieldCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors;
  private final SwerveRequest.ApplyRobotSpeeds m_chassisSpeedsRequest =
    new SwerveRequest.ApplyRobotSpeeds()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors;

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   * <p>
   * This constructs the underlying hardware devices, so users should not construct
   * the devices themselves. If they need the devices, they can access them through
   * getters in the classes.
   *
   * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
   * @param modules               Constants for each specific module
   */
  public CommandSwerveDrivetrain(
    SwerveDrivetrainConstants drivetrainConstants,
    SwerveModuleConstants<?, ?, ?>... modules
  ) {
    super(drivetrainConstants, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
  }

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   * <p>
   * This constructs the underlying hardware devices, so users should not construct
   * the devices themselves. If they need the devices, they can access them through
   * getters in the classes.
   *
   * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
   * @param odometryUpdateFrequency The frequency to run the odometry loop. If
   *                                unspecified or set to 0 Hz, this is 250 Hz on
   *                                CAN FD, and 100 Hz on CAN 2.0.
   * @param modules                 Constants for each specific module
   */
  public CommandSwerveDrivetrain(
    SwerveDrivetrainConstants drivetrainConstants,
    double odometryUpdateFrequency,
    SwerveModuleConstants<?, ?, ?>... modules
  ) {
    super(drivetrainConstants, odometryUpdateFrequency, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
  }

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   * <p>
   * This constructs the underlying hardware devices, so users should not construct
   * the devices themselves. If they need the devices, they can access them through
   * getters in the classes.
   *
   * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
   * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
   *                                  unspecified or set to 0 Hz, this is 250 Hz on
   *                                  CAN FD, and 100 Hz on CAN 2.0.
   * @param odometryStandardDeviation The standard deviation for odometry calculation
   *                                  in the form [x, y, theta]ᵀ, with units in meters
   *                                  and radians
   * @param visionStandardDeviation   The standard deviation for vision calculation
   *                                  in the form [x, y, theta]ᵀ, with units in meters
   *                                  and radians
   * @param modules                   Constants for each specific module
   */
  public CommandSwerveDrivetrain(
    SwerveDrivetrainConstants drivetrainConstants,
    double odometryUpdateFrequency,
    Matrix<N3, N1> odometryStandardDeviation,
    Matrix<N3, N1> visionStandardDeviation,
    SwerveModuleConstants<?, ?, ?>... modules
  ) {
    super(
      drivetrainConstants,
      odometryUpdateFrequency,
      odometryStandardDeviation,
      visionStandardDeviation,
      modules
    );
    if (Utils.isSimulation()) {
      startSimThread();
    }
  }

  /**
   * Returns a command that applies the specified control request to this swerve drivetrain.
   *
   * @param request Function returning the request to apply
   * @return Command to run
   */
  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  /**
   * Runs the SysId Quasistatic test in the given direction for the routine
   * specified by {@link #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Quasistatic test
   * @return Command to run
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.quasistatic(direction);
  }

  /**
   * Runs the SysId Dynamic test in the given direction for the routine
   * specified by {@link #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Dynamic test
   * @return Command to run
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.dynamic(direction);
  }

  @Override
  public void periodic() {
    // if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
    //   DriverStation.getAlliance()
    //     .ifPresent(allianceColor -> {
    //       setOperatorPerspectiveForward(
    //         allianceColor == Alliance.Red
    //           ? kRedAlliancePerspectiveRotation
    //           : kBlueAlliancePerspectiveRotation
    //       );
    //       m_hasAppliedOperatorPerspective = true;
    //     });
    // }
  }

  private void startSimThread() {
    m_lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier = new Notifier(() -> {
      final double currentTime = Utils.getCurrentTimeSeconds();
      double deltaTime = currentTime - m_lastSimTime;
      m_lastSimTime = currentTime;

      /* use the measured time delta, get battery voltage from WPILib */
      updateSimState(deltaTime, RobotController.getBatteryVoltage());
    });
    m_simNotifier.startPeriodic(kSimLoopPeriod);
  }

  /**
   * The method to use for robot relative driving.
   *
   * @param velocityXSpeedMetersPerSecond The desired speed on the X axis in meters per second.
   * @param velocityYSpeedMetersPerSecond The desired speed on the X axis in meters per second.
   * @param rotationRateRadiansPerSecond The desired rotation rate in radians per second.
   */
  public void driveRobotRelative(
    double velocityXMetersPerSecond,
    double velocityYMetersPerSecond,
    double rotationRateRadiansPerSecond
  ) {
    setControl(
      m_robotRelative
        .withVelocityX(velocityXMetersPerSecond)
        .withVelocityY(velocityYMetersPerSecond)
        .withRotationalRate(rotationRateRadiansPerSecond)
    );
  }

  /**
   * The method to use for field relative driving.
   *
   * @param velocityXSpeedMetersPerSecond The desired speed on the X axis in meters per second.
   * @param velocityYSpeedMetersPerSecond The desired speed on the X axis in meters per second.
   * @param rotationRateRadiansPerSecond The desired rotation rate in radians per second.
   */
  public void driveFieldRelative(
    double velocityXMetersPerSecond,
    double velocityYMetersPerSecond,
    double rotationRateRadiansPerSecond
  ) {
    setControl(
      m_fieldRelative
        .withVelocityX(velocityXMetersPerSecond)
        .withVelocityY(velocityYMetersPerSecond)
        .withRotationalRate(rotationRateRadiansPerSecond)
    );
  }

  public void followChoreoPath(SwerveSample sample) {
    Pose2d pose = getFieldRelativePose2d();
    CHOREO.ROTATION_CONTROLLER.enableContinuousInput(-Math.PI, Math.PI);

    var targetSpeeds = sample.getChassisSpeeds();
    targetSpeeds.vxMetersPerSecond += CHOREO.X_CONTROLLER.calculate(
      pose.getX(),
      sample.x
    );
    targetSpeeds.vyMetersPerSecond += CHOREO.Y_CONTROLLER.calculate(
      pose.getY(),
      sample.y
    );
    targetSpeeds.omegaRadiansPerSecond += CHOREO.ROTATION_CONTROLLER.calculate(
      pose.getRotation().getRadians(),
      sample.heading
    );
    setControl(
      m_chassisSpeedsRequest
        .withSpeeds(targetSpeeds)
        .withWheelForceFeedforwardsX(sample.moduleForcesX())
        .withWheelForceFeedforwardsY(sample.moduleForcesY())
    );
  }

  /**
   * Gets the pose, with no flipping to compensate for alliance.
   * @return The field relative pose.
   */
  public Pose2d getFieldRelativePose2d() {
    return super.getState().Pose;
  }

  /**
   * The pose with flipping to ensure it is always on the blue origin.
   * @return The pose flipped to ensure it is on the blue origin.
   */
  public Pose2d getAllianceRelativePose2d() {
    var curPose = getFieldRelativePose2d();
    return Robot.alliance == Alliance.Blue
      ? curPose
      : ChoreoAllianceFlipUtil.flip(curPose);
  }

  /**
   * Sets the pose staright as you input it, with no flipping to compensate for alliance.
   * @param poseToSet The pose it will set.
   */
  public void setFieldRelativePose2d(Pose2d poseToSet) {
    super.resetPose(poseToSet);
  }

  /**
   * Sets the pose relative to the alliance, if alliance is red, flips the pose.
   * @param poseToSet The pose to set. Its origin must be on the blue origin to set correctly.
   */
  public void setAllianceRelativePose2d(Pose2d poseToSet) {
    super.resetPose(
      Robot.alliance == Alliance.Blue
        ? poseToSet
        : ChoreoAllianceFlipUtil.flip(poseToSet)
    );
  }

  public ChassisSpeeds getCurrentChassisSpeeds() {
    return super.getState().Speeds;
  }

  public LinearVelocity getXVelocity() {
    return Units.MetersPerSecond.of(
      getCurrentChassisSpeeds().vxMetersPerSecond
    );
  }

  public LinearVelocity getYVelocity() {
    return Units.MetersPerSecond.of(
      getCurrentChassisSpeeds().vyMetersPerSecond
    );
  }

  public LinearVelocity getTranslationalVelocity() {
    var speeds = getCurrentChassisSpeeds();
    var xVel = Math.abs(speeds.vxMetersPerSecond);
    var yVel = Math.abs(speeds.vyMetersPerSecond);
    var translationalVelocity = Math.sqrt(
      Math.pow(xVel, 2) + Math.pow(yVel, 2)
    ); // A^2 + B^2 = C^2
    return Units.FeetPerSecond.of(translationalVelocity);
  }

  public AngularVelocity getThetaVelocity() {
    return Units.RadiansPerSecond.of(
      getCurrentChassisSpeeds().omegaRadiansPerSecond
    );
  }

  public void stopMotors() {
    driveFieldRelative(0, 0, 0);
    for (SwerveModule<TalonFX, TalonFX, CANcoder> module : super.getModules()) {
      module.getDriveMotor().stopMotor(); // anti-jingle
      module.getSteerMotor().stopMotor(); // remove to bring back the jingle (dont do it)
    }
  }

  public double getYaw() {
    return getPigeon2().getYaw().getValueAsDouble();
  }

  // Pigeon is rotated 90 degrees so pitch and roll are flipped
  public double getRoll() {
    return getPigeon2().getPitch().getValueAsDouble();
  }

  // Pigeon is rotated 90 degrees so pitch and roll are flipped
  public double getPitch() {
    return getPigeon2().getRoll().getValueAsDouble();
  }
}

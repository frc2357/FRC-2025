package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.SWERVE.FACING_ANGLE_D;
import static frc.robot.Constants.SWERVE.FACING_ANGLE_I;
import static frc.robot.Constants.SWERVE.FACING_ANGLE_P;

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
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
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
@SuppressWarnings("unused")
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

  private final SwerveRequest.RobotCentric m_robotRelative =
    new SwerveRequest.RobotCentric()
      .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive motors;

  private final SwerveRequest.FieldCentric m_fieldRelative =
    new SwerveRequest.FieldCentric()
      .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive motors;

  private final SwerveRequest.ApplyFieldSpeeds m_fieldSpeedsRequest =
    new SwerveRequest.ApplyFieldSpeeds()
      .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive motors;

  private final SwerveRequest.FieldCentricFacingAngle m_fieldCentricFacingAngle =
    new SwerveRequest.FieldCentricFacingAngle()
      .withDriveRequestType(DriveRequestType.Velocity)
      .withHeadingPID(FACING_ANGLE_P, FACING_ANGLE_I, FACING_ANGLE_D); // Use open-loop control for drive motors;

  private final SwerveRequest.SwerveDriveBrake m_brakeRequest =
    new SwerveRequest.SwerveDriveBrake();

  private Twist2d m_fieldRelativeRobotVelocity = new Twist2d();

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

  @Override
  public void periodic() {
    updateFieldVelocity();
  }

  public void setOperatorPerspectiveForward(DriverStation.Alliance alliance) {
    setOperatorPerspectiveForward(
      alliance == Alliance.Red
        ? kRedAlliancePerspectiveRotation
        : kBlueAlliancePerspectiveRotation
    );
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
   * @param velocityXMetersPerSecond The desired speed on the X axis in meters per second.
   * @param velocityYMetersPerSecond The desired speed on the Y axis in meters per second.
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
   * @param velocityXMetersPerSecond The desired speed on the X axis in meters per second.
   * @param velocityYMetersPerSecond The desired speed on the Y axis in meters per second.
   * @param rotationRateRadiansPerSecond The desired rotation rate in radians per second.
   * @param perspective The perspective to use for field relative driving.
   */
  public void driveFieldRelative(
    double velocityXMetersPerSecond,
    double velocityYMetersPerSecond,
    double rotationRateRadiansPerSecond,
    ForwardPerspectiveValue perspective
  ) {
    setControl(
      m_fieldRelative
        .withVelocityX(velocityXMetersPerSecond)
        .withVelocityY(velocityYMetersPerSecond)
        .withRotationalRate(rotationRateRadiansPerSecond)
        .withForwardPerspective(perspective)
    );
  }

  /**
   * The method to use for field relative driving.
   *
   * @param velocityXMetersPerSecond The desired speed on the X axis in meters per second.
   * @param velocityYMetersPerSecond The desired speed on the Y axis in meters per second.
   * @param rotationRateRadiansPerSecond The desired rotation rate in radians per second.
   */
  public void driveFieldRelative(
    double velocityXMetersPerSecond,
    double velocityYMetersPerSecond,
    double rotationRateRadiansPerSecond
  ) {
    driveFieldRelative(
      velocityXMetersPerSecond,
      velocityYMetersPerSecond,
      rotationRateRadiansPerSecond,
      ForwardPerspectiveValue.OperatorPerspective
    );
  }

  /**
   * The method to use for target angle driving.
   *
   * @param velocityXMetersPerSecond The desired speed on the X axis in meters per second.
   * @param velocityYMetersPerSecond The desired speed on the Y axis in meters per second.
   * @param targetAngle The target angle.
   */
  public void driveTargetAngle(
    double velocityXMetersPerSecond,
    double velocityYMetersPerSecond,
    Rotation2d targetAngle
  ) {
    setControl(
      m_fieldCentricFacingAngle
        .withVelocityX(velocityXMetersPerSecond)
        .withVelocityY(velocityYMetersPerSecond)
        .withTargetDirection(targetAngle)
    );
  }

  /**
   * The method to use to set the drivetrain into brake mode.
   */
  public void driveBrake() {
    setControl(m_brakeRequest);
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
      m_fieldSpeedsRequest
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

  public Pose2d makePoseAllianceRelative(Pose2d pose) {
    return Robot.alliance == Alliance.Blue
      ? pose
      : ChoreoAllianceFlipUtil.flip(pose);
  }

  public Pose2d flipYAxis(Pose2d poseToFlip) {
    return new Pose2d(
      poseToFlip.getX(),
      ChoreoAllianceFlipUtil.flipY(poseToFlip.getY()),
      poseToFlip.getRotation()
    );
  }

  public Pose2d flipXAxis(Pose2d poseToFlip) {
    return new Pose2d(
      ChoreoAllianceFlipUtil.flipX(poseToFlip.getX()),
      poseToFlip.getY(),
      poseToFlip.getRotation()
    );
  }

  /**
   * Sets the pose straight as you input it, with no flipping to compensate for alliance.
   * @param poseToSet The pose it will set.
   */
  public void setFieldRelativePose2d(Pose2d poseToSet) {
    super.resetPose(poseToSet);
  }

  /**
   * Sets the translation straight as you input it, with no flipping to compensate for alliance.
   * @param translationToSet The translation it will set.
   */
  public void setFieldRelativeTranslation2d(Translation2d translationToSet) {
    super.resetTranslation(translationToSet);
  }

  /**
   * Sets the translation straight as you input it, with no flipping to compensate for alliance.
   * @param translationToSet The translation it will set.
   */
  public void setAllianceRelativeTranslation2d(Translation2d translationToSet) {
    super.resetTranslation(
      Robot.alliance == Alliance.Blue
        ? translationToSet
        : ChoreoAllianceFlipUtil.flip(translationToSet)
    );
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

  public void resetHeading(Rotation2d heading) {
    super.resetRotation(heading);
  }

  public void resetTranslation(Translation2d translation) {
    super.resetTranslation(translation);
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

  public LinearVelocity getAbsoluteTranslationalVelocity() {
    var speeds = getCurrentChassisSpeeds();
    var xVel = Math.abs(speeds.vxMetersPerSecond);
    var yVel = Math.abs(speeds.vyMetersPerSecond);
    var translationalVelocity = Math.sqrt(
      Math.pow(xVel, 2) + Math.pow(yVel, 2)
    ); // A^2 + B^2 = C^2
    return Units.MetersPerSecond.of(translationalVelocity);
  }

  public AngularVelocity getAngularVelocity() {
    return Units.RadiansPerSecond.of(getState().Speeds.omegaRadiansPerSecond);
  }

  public void stopMotors() {
    driveFieldRelative(0, 0, 0);
    for (SwerveModule<TalonFX, TalonFX, CANcoder> module : super.getModules()) {
      module.getDriveMotor().stopMotor(); // anti-jingle
      module.getSteerMotor().stopMotor(); // remove to bring back the jingle (dont do it)
    }
  }

  public Angle getYaw() {
    return getPigeon2().getYaw().getValue();
  }

  // Pigeon is rotated 90 degrees so pitch and roll are flipped
  public double getRoll() {
    return getPigeon2().getPitch().getValueAsDouble();
  }

  // Pigeon is rotated 90 degrees so pitch and roll are flipped
  public double getPitch() {
    return getPigeon2().getRoll().getValueAsDouble();
  }

  public Twist2d getRobotVelocity() {
    return new Twist2d(
      getXVelocity().in(Units.MetersPerSecond),
      getYVelocity().in(Units.MetersPerSecond),
      getPigeon2()
        .getAngularVelocityZWorld()
        .getValue()
        .in(Units.RadiansPerSecond)
    );
  }

  public Twist2d getFieldRelativeRobotVelocity() {
    return m_fieldRelativeRobotVelocity;
  }

  private void updateFieldVelocity() {
    Translation2d linearFieldVelocity = new Translation2d(
      getXVelocity().in(Units.MetersPerSecond),
      getYVelocity().in(Units.MetersPerSecond)
    ).rotateBy(getFieldRelativePose2d().getRotation());

    m_fieldRelativeRobotVelocity = new Twist2d(
      linearFieldVelocity.getX(),
      linearFieldVelocity.getY(),
      getPigeon2()
        .getAngularVelocityZWorld()
        .getValue()
        .in(Units.RadiansPerSecond)
    );
  }

  /**
   * @return A list of module states in the order Front Left, Front Right, Back Left, Back Right
   */
  public SwerveModuleState[] getModuleStates() {
    return super.getState().ModuleStates;
  }
}

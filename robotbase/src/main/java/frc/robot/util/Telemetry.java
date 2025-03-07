package frc.robot.util;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;

public class Telemetry {

  /* What to publish over networktables for telemetry */
  static NetworkTableInstance inst = NetworkTableInstance.getDefault();

  /* Robot pose for field positioning */
  static NetworkTable table = inst.getTable("Pose");
  static DoubleArrayPublisher fieldPub = table
    .getDoubleArrayTopic("robotPose")
    .publish();
  static StringPublisher fieldTypePub = table.getStringTopic(".type").publish();

  /* Robot speeds for general checking */
  NetworkTable driveStats = inst.getTable("Drive");
  DoublePublisher velocityX = driveStats.getDoubleTopic("Velocity X").publish();
  DoublePublisher velocityY = driveStats.getDoubleTopic("Velocity Y").publish();
  DoublePublisher speed = driveStats.getDoubleTopic("Speed").publish();
  DoublePublisher odomPeriod = driveStats
    .getDoubleTopic("Odometry Period")
    .publish();

  /* Keep a reference of the last pose to calculate the speeds */
  Pose2d m_lastPose = new Pose2d();
  double lastTime = Utils.getCurrentTimeSeconds();

  /* Accept the swerve drive state and telemeterize it to smartdashboard */
  public void telemeterize(SwerveDriveState state) {
    /* Telemeterize the pose */
    Pose2d pose = state.Pose;
    fieldTypePub.set("Field2d");
    fieldPub.set(
      new double[] { pose.getX(), pose.getY(), pose.getRotation().getDegrees() }
    );

    /* Telemeterize the robot's general speeds */
    double currentTime = Utils.getCurrentTimeSeconds();
    double diffTime = currentTime - lastTime;
    lastTime = currentTime;
    Translation2d distanceDiff = pose.minus(m_lastPose).getTranslation();
    m_lastPose = pose;

    Translation2d velocities = distanceDiff.div(diffTime);

    speed.set(velocities.getNorm());
    velocityX.set(velocities.getX());
    velocityY.set(velocities.getY());
    odomPeriod.set(state.OdometryPeriod);
    // Log the odometry pose as a double array
    SignalLogger.writeDoubleArray(
      "odometry",
      new double[] { pose.getX(), pose.getY(), pose.getRotation().getDegrees() }
    );
    // Log the odometry period with units of "seconds"
    SignalLogger.writeDouble("odom period", state.OdometryPeriod, "seconds");

    // Making this compatible with AdvantageKit Swerve Module, per the documentation, accepts an
    // eight element array of angle, speed elements
    SignalLogger.writeDoubleArray(
      "SwerveState",
      new double[] {
        state.ModuleStates[0].angle.getDegrees(),
        state.ModuleStates[0].speedMetersPerSecond,
        state.ModuleStates[1].angle.getDegrees(),
        state.ModuleStates[1].speedMetersPerSecond,
        state.ModuleStates[2].angle.getDegrees(),
        state.ModuleStates[2].speedMetersPerSecond,
        state.ModuleStates[3].angle.getDegrees(),
        state.ModuleStates[3].speedMetersPerSecond,
      }
    );
    SignalLogger.writeDoubleArray(
      "SwerveTarget",
      new double[] {
        state.ModuleTargets[0].angle.getDegrees(),
        state.ModuleTargets[0].speedMetersPerSecond,
        state.ModuleTargets[1].angle.getDegrees(),
        state.ModuleTargets[1].speedMetersPerSecond,
        state.ModuleTargets[2].angle.getDegrees(),
        state.ModuleTargets[2].speedMetersPerSecond,
        state.ModuleTargets[3].angle.getDegrees(),
        state.ModuleTargets[3].speedMetersPerSecond,
      }
    );
  }
}

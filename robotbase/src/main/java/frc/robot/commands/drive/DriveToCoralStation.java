package frc.robot.commands.drive;

import static frc.robot.Constants.FIELD.CORAL_STATION.LEFT_STATION_DESIRED_SLOT;
import static frc.robot.Constants.FIELD.CORAL_STATION.RIGHT_STATION_DESIRED_SLOT;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.Utility;

public class DriveToCoralStation extends DriveToPoseHandler {

  public enum StationToGoTo {
    LeftSide,
    RightSide,
    Fastest,
  }

  private StationToGoTo m_desiredStation;

  public DriveToCoralStation(
    StationToGoTo desiredStation,
    RouteAroundReef routeAroundReef
  ) {
    this(desiredStation, routeAroundReef, null);
  }

  public DriveToCoralStation(
    StationToGoTo desiredStation,
    RouteAroundReef routeAroundReef,
    Command finalApproachCommand
  ) {
    super(routeAroundReef, finalApproachCommand);
    m_desiredStation = desiredStation != null
      ? desiredStation
      : StationToGoTo.Fastest;
  }

  @Override
  protected Pose2d getNewTarget(Pose2d currTarget, Pose2d currPose) {
    m_finalGoal = getDesiredTarget(currPose);
    return super.getNewTarget(currTarget, currPose);
  }

  private Pose2d getDesiredTarget(Pose2d currPose) {
    switch (m_desiredStation) {
      case RightSide:
        return RIGHT_STATION_DESIRED_SLOT;
      case LeftSide:
        return LEFT_STATION_DESIRED_SLOT;
      case Fastest:
      default:
        double leftStationDist = Utility.findDistanceBetweenPoses(
          currPose,
          LEFT_STATION_DESIRED_SLOT
        );
        double rightStationDist = Utility.findDistanceBetweenPoses(
          currPose,
          RIGHT_STATION_DESIRED_SLOT
        );
        return leftStationDist <= rightStationDist
          ? LEFT_STATION_DESIRED_SLOT
          : RIGHT_STATION_DESIRED_SLOT;
    }
  }
}

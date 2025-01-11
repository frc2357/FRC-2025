package frc.robot.state;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class RobotState {
  public static enum State {
    INIT, // Robot is initializing
    DISABLED, // Robot is in disabled mode and no alliance is selected
  }

  public static enum DriveControlState {
    FIELD_RELATIVE, // Manual control of the robot is field relative
    ROBOT_RELATIVE, // Manual control of the robot is robot centric
    TARGET_LOCK
  }

  private Alliance m_alliance;
  private State m_currentState;
  private DriveControlState m_currentDriveControlState;

  public RobotState() {
    m_alliance = null;
    m_currentState = State.INIT;
    m_currentDriveControlState = DriveControlState.FIELD_RELATIVE;
  }

  public boolean isRobot(State state) {
    return m_currentState == state;
  }

  public Alliance getAlliance() {
    return m_alliance;
  }

  public void robotInit() {
    setState(State.INIT);
  }

  public void disabledInit() {
    setState(State.DISABLED);
  }

  public void setState(State newState) {
    setCurrentState(newState);
  }

  public boolean isFieldRelative() {
    return m_currentDriveControlState == DriveControlState.FIELD_RELATIVE;
  }

  public boolean isTargetLock() {
    return getDriveControlState() == DriveControlState.TARGET_LOCK;
  }

  public DriveControlState getDriveControlState() {
    return m_currentDriveControlState;
  }

  public void setDriveControlState(DriveControlState driveControlState) {
    m_currentDriveControlState = driveControlState;
  }

  public void setAlliance(Alliance alliance) {
    m_alliance = alliance;
  }

  private void setCurrentState(State newState) {
    m_currentState = newState;
  }

}

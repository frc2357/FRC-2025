// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.Constants.FIELD.REEF.BLUE_REEF_TAGS;
import static frc.robot.Constants.PHOTON_VISION.*;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.cscore.OpenCvLoader;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.CLIMBER_PIVOT;
import frc.robot.Constants.PHOTON_VISION;
import frc.robot.Constants.PHOTON_VISION.BACK_LEFT_CAM;
import frc.robot.Constants.PHOTON_VISION.BACK_RIGHT_CAM;
import frc.robot.Constants.SWERVE;
import frc.robot.commands.StopAllMotors;
import frc.robot.commands.climberPivot.ClimberPivotSetSpeed;
import frc.robot.commands.coralRunner.CoralRunnerSetSpeed;
import frc.robot.commands.drive.DefaultDrive;
import frc.robot.commands.drive.DriveSetCoast;
import frc.robot.commands.elevator.ElevatorHoldPosition;
import frc.robot.commands.elevator.ElevatorSetDistance;
import frc.robot.commands.laterator.LateratorHoldHome;
import frc.robot.commands.laterator.LateratorSetDistance;
import frc.robot.commands.rumble.ClearButtonboard;
import frc.robot.commands.util.InitRobotCommand;
import frc.robot.controls.Buttonboard;
import frc.robot.controls.CodriverControls;
import frc.robot.controls.DriverControls;
import frc.robot.controls.controllers.CommandButtonboardController;
import frc.robot.generated.TunerConstants;
import frc.robot.networkTables.*;
import frc.robot.subsystems.*;
import frc.robot.util.Telemetry;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
@SuppressWarnings("unused")
public class Robot extends TimedRobot {

  // Subsystems
  public static CommandSwerveDrivetrain swerve;
  public static Elevator elevator;
  public static Laterator laterator;
  public static CoralRunner coralRunner;
  public static AlgaeKnocker algaeKnocker;
  public static ClimberPivot climberPivot;
  public static ClimberWinch climberWinch;
  public static CameraManager camManager;
  public static PhotonVisionCamera backRightCam;
  public static PhotonVisionCamera backLeftCam;

  // state
  public static Alliance alliance = null;

  // Commands
  public static DriverControls driverControls;
  public static CodriverControls codriverControls;
  public static Buttonboard buttonboard;

  // PhotonVision Cameras
  private static PhotonVisionCamera frontCam;
  private static PhotonVisionCamera backCam;
  private static PhotonVisionCamera leftCam;
  private static PhotonVisionCamera rightCam;

  private Command m_autonomousCommand;
  private SequentialCommandGroup m_setCoastOnDisable;

  private AutoChooserManager m_autoChooserManager;
  private SignalLoggerManager m_SignalLoggerManager;
  private boolean m_didOpenCVLoad = false;

  /**
   * This function is run when the robot is first started up
   * and should be used for any initialization code.
   */
  public Robot() {
    DriverStation.silenceJoystickConnectionWarning(
      !DriverStation.isFMSAttached()
    );
    SmartDashboard.putBoolean("Toggle Pose Estimation", true);

    // Define subsystems
    swerve = TunerConstants.createDrivetrain();
    elevator = new Elevator();
    laterator = new Laterator();
    coralRunner = new CoralRunner();
    algaeKnocker = new AlgaeKnocker();

    camManager = new CameraManager();
    backRightCam = camManager.createCamera(
      BACK_RIGHT_CAM.NAME,
      BACK_RIGHT_CAM.ROBOT_TO_CAM_TRANSFORM
    );
    backLeftCam = camManager.createCamera(
      BACK_LEFT_CAM.NAME,
      BACK_LEFT_CAM.ROBOT_TO_CAM_TRANSFORM
    );
    // leftCam = camManager.createCamera(
    //   LEFT_CAM.NAME,
    //   LEFT_CAM.ROBOT_TO_CAM_TRANSFORM
    // );
    // rightCam = camManager.createCamera(
    //   RIGHT_CAM.NAME,
    //   RIGHT_CAM.ROBOT_TO_CAM_TRANSFORM
    // );

    // Define controls
    buttonboard = new Buttonboard(
      new CommandButtonboardController(
        Constants.CONTROLLER.BUTTONBOARD_CONTROLLER_PORT
      )
    );
    driverControls = new DriverControls(
      new CommandXboxController(Constants.CONTROLLER.DRIVE_CONTROLLER_PORT),
      Constants.CONTROLLER.DRIVE_CONTROLLER_DEADBAND
    );
    codriverControls = new CodriverControls(
      new CommandXboxController(Constants.CONTROLLER.CODRIVER_CONTROLLER_PORT),
      Constants.CONTROLLER.CODRIVE_CONTROLLER_DEADBAND
    );

    // Define network table tools
    m_autoChooserManager = new AutoChooserManager();
    m_SignalLoggerManager = new SignalLoggerManager();

    SmartDashboard.putData("Buttonboard", buttonboard);
    SmartDashboard.putData("ClearButtonboard", new ClearButtonboard());
    SmartDashboard.putData("Signal Logger", m_SignalLoggerManager);

    // Logging
    DataLogManager.logNetworkTables(true); // enable/disable automatic NetworksTable Logging
    DataLogManager.start("", "", 1.0); // defaults, flush to flash every 1 seconds
    DriverStation.startDataLog(DataLogManager.getLog());

    // Setup logging swerve pose and state for viewing in Advantage Scope
    Robot.swerve.registerTelemetry(new Telemetry()::telemeterize);
    // Setup commands
    swerve.setDefaultCommand(new DefaultDrive());
    elevator.setDefaultCommand(new ElevatorHoldPosition());

    new InitRobotCommand().schedule();

    m_setCoastOnDisable = new WaitCommand(SWERVE.TIME_TO_COAST).andThen(
      new DriveSetCoast()
    );

    // climberPivot.setDefaultCommand(
    //   new ClimberPivotSetSpeed(CLIMBER_PIVOT.HOLD_AGAINST_WINCH_SPEED)
    // );

    // Update sensors at a faster rate
    addPeriodic(
      () -> {
        Robot.coralRunner.updateSensors();
        Robot.laterator.updateSensors();
      },
      0.005,
      0.003
    );
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    camManager.updateAllCameras();
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    new StopAllMotors().schedule();
    m_setCoastOnDisable.schedule();
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    swerve.configNeutralMode(NeutralModeValue.Brake);
    m_autonomousCommand = m_autoChooserManager.getSelectedCommandScheduler();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    m_setCoastOnDisable.cancel();

    swerve.configNeutralMode(NeutralModeValue.Brake);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

}

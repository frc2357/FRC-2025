// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.PHOTON_VISION.*;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.cscore.OpenCvLoader;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.SWERVE;
import frc.robot.commands.drive.DefaultDrive;
import frc.robot.commands.drive.DriveSetCoast;
import frc.robot.commands.rumble.ClearButtonboard;
import frc.robot.commands.util.InitRobotCommand;
import frc.robot.controls.Buttonboard;
import frc.robot.controls.CodriverControls;
import frc.robot.controls.DriverControls;
import frc.robot.controls.controllers.CommandButtonboardController;
import frc.robot.generated.TunerConstants;
import frc.robot.networkTables.*;
import frc.robot.subsystems.*;
import frc.robot.util.ElasticFieldManager;
import frc.robot.util.Telemetry;
import org.photonvision.proto.Photon;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
@SuppressWarnings("unused")
public class Robot extends TimedRobot {

  public static CommandSwerveDrivetrain swerve;
  public static Elevator elevator;
  public static Laterator laterator;
  public static CoralRunner coralRunner;
  public static AlgaeRunner algaeRunner;
  public static AlgaeKnocker algaeKnocker;
  public static AlgaePivot algaePivot;
  public static Climber climber;
  public static PhotonVisionCamera frontCam;
  public static PhotonVisionCamera backCam;
  public static PhotonVisionCamera leftCam;
  public static PhotonVisionCamera rightCam;
  public static DriverControls driverControls;
  public static CodriverControls codriverControls;
  public static Buttonboard buttonboard;
  public static ElasticFieldManager elasticFieldManager;

  public static Alliance alliance = null;

  private Command m_autonomousCommand;
  private SequentialCommandGroup m_setCoastOnDisable;
  private AutoChooserManager m_autoChooserManager;
  private SignalLoggerManager m_SignalLoggerManager;
  private boolean m_didOpenCVLoad = false;

  @SuppressWarnings("unused")
  private SysIdChooser m_sysIdChooser;

  /**
   * This function is run when the robot is first started up
   * and should be used for any initialization code.
   */
  public Robot() {
    // forces OpenCV to load. Dont remove this.
    for (int i = 0; i < 3; i++) {
      try {
        OpenCvLoader.forceLoad();
        m_didOpenCVLoad = true;
        break;
      } catch (Exception e) {
        if (i > 2) {
          System.err.println(
            "\n\nOpenCV Load FAILED! ******* TELL MAX ASAP! *******"
          );
        }
      }
    }
    DriverStation.silenceJoystickConnectionWarning(
      !DriverStation.isFMSAttached()
    ); // TODO: turn this off at comp, just in case.

    // Define subsystems
    swerve = TunerConstants.createDrivetrain();
    elevator = new Elevator();
    laterator = new Laterator();
    coralRunner = new CoralRunner();
    algaeRunner = new AlgaeRunner();
    algaeKnocker = new AlgaeKnocker();
    algaePivot = new AlgaePivot();
    // climber = new Climber();
    frontCam = new PhotonVisionCamera(
      FRONT_CAM.NAME,
      FRONT_CAM.ROBOT_TO_CAM_TRANSFORM,
      FRONT_CAM.CAMERA_MATRIX,
      FRONT_CAM.DIST_COEFFS
    );
    backCam = new PhotonVisionCamera(
      BACK_CAM.NAME,
      BACK_CAM.ROBOT_TO_CAM_TRANSFORM,
      BACK_CAM.CAMERA_MATRIX,
      BACK_CAM.DIST_COEEFS
    );
    leftCam = new PhotonVisionCamera(
      LEFT_CAM.NAME,
      LEFT_CAM.ROBOT_TO_CAM_TRANSFORM,
      LEFT_CAM.CAMERA_MATRIX,
      LEFT_CAM.DIST_COEEFS
    );
    rightCam = new PhotonVisionCamera(
      RIGHT_CAM.NAME,
      RIGHT_CAM.ROBOT_TO_CAM_TRANSFORM,
      RIGHT_CAM.CAMERA_MATRIX,
      RIGHT_CAM.DIST_COEEFS
    );
    // if openCV fails to load, we cant use our normal strategies, and must change them accordingly.
    if (!m_didOpenCVLoad) {
      PhotonVisionCamera.setPrimaryStrategy(PRIMARY_STRAT_FOR_FAILED_LOAD);
      PhotonVisionCamera.setPrimaryStrategy(FALLBACK_STRAT_FOR_FAILED_LOAD);
    }
    elasticFieldManager = new ElasticFieldManager();
    elasticFieldManager.setupSwerveField();

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
    m_sysIdChooser = new SysIdChooser();
    m_SignalLoggerManager = new SignalLoggerManager();
    elasticFieldManager = new ElasticFieldManager();
    elasticFieldManager.setupSwerveField();

    SmartDashboard.putData("Buttonboard", buttonboard);
    SmartDashboard.putData("ClearButtonboard", new ClearButtonboard());
    SmartDashboard.putData("Signal Logger", m_SignalLoggerManager);

    elasticFieldManager = new ElasticFieldManager();
    elasticFieldManager.setupSwerveField();
    elasticFieldManager.setupShooterField();

    // Logging
    DataLogManager.logNetworkTables(true); // enable/disable automatic NetworksTable Logging
    DataLogManager.start("", "", 1.0); // defaults, flush to flash every 1 seconds
    DriverStation.startDataLog(DataLogManager.getLog());

    // Setup logging swerve pose and state for viewing in Advantage Scope
    Robot.swerve.registerTelemetry(new Telemetry()::telemeterize);

    // Setup commands
    swerve.setDefaultCommand(new DefaultDrive());
    new InitRobotCommand().schedule();

    m_setCoastOnDisable = new WaitCommand(SWERVE.TIME_TO_COAST).andThen(
      new DriveSetCoast()
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
    PhotonVisionCamera.updateAllCameras();
    elasticFieldManager.swerveFieldRep.setRobotPose(
      swerve.getFieldRelativePose2d()
    );
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
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

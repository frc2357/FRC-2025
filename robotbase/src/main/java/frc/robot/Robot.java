// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.PHOTON_VISION;
import frc.robot.commands.auto.Autos;
import frc.robot.commands.drive.DefaultDrive;
import frc.robot.commands.util.InitRobotCommand;
import frc.robot.controls.DriverControls;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaePivot;
import frc.robot.subsystems.AlgaeRunner;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralRunner;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Laterator;
import frc.robot.subsystems.PhotonVisionCamera;
import java.util.function.Consumer;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {

  private Command m_autonomousCommand;

  public static CommandSwerveDrivetrain swerve;
  public static Elevator elevator;
  public static Laterator laterator;
  public static CoralRunner coralRunner;
  public static AlgaeRunner algaeRunner;
  public static AlgaePivot algaePivot;

  public static AutoChooserManager autoChooserManager;
  public static Autos autos;
  public static DriverControls driverControls;

  public static Alliance alliance = null;

  public static Field2d shooterFieldRepresentation;
  public static Field2d swerveFieldRepresentation;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    DriverStation.silenceJoystickConnectionWarning(true); //TODO: turn this off at comp, just in case.

    swerve = TunerConstants.createDrivetrain();
    // elevator = new Elevator();
    // laterator = new Laterator();
    // coralRunner = new CoralRunner();
    // algaeRunner = new AlgaeRunner();
    // algaePivot = new AlgaePivot(); // commented out because they are currently NOT on the robot, and it will not run without them commented out.

    autos = new Autos();
    autoChooserManager = new AutoChooserManager();

    driverControls = new DriverControls(
      new CommandXboxController(Constants.CONTROLLER.DRIVE_CONTROLLER_PORT),
      Constants.CONTROLLER.DRIVE_CONTROLLER_DEADBAND
    );

    swerve.setDefaultCommand(new DefaultDrive());
    new InitRobotCommand().schedule();
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
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = autoChooserManager.getSelectedCommandScheduler();

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

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoRoutine;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.auto.Autos;
import java.util.Map;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {

  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public static AutoChooser autoChooser;

  private Autos m_autos;

  private Map<String, Command> m_autoCommandsToBind;
  private Map<String, AutoRoutine> m_autoRoutinesToBind;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    m_robotContainer = new RobotContainer();

    autoChooser = new AutoChooser();

    m_autos = new Autos();

    SendableBuilderImpl autoChooserBuilder = new SendableBuilderImpl();
    autoChooserBuilder.setTable(
      NetworkTableInstance.getDefault().getTable("SmartDashboard/Auto chooser")
    );
    autoChooser.initSendable(autoChooserBuilder);
    // The map of named commands we use in choreo
    m_autoCommandsToBind = Map.of();

    // The map of auto routines that will show up on the auto command chooser.
    m_autoRoutinesToBind = Map.of();

    m_autoCommandsToBind.forEach((String name, Command command) -> {
      autoChooser.addCmd(name, () -> command);
    });
    m_autoRoutinesToBind.forEach((String name, AutoRoutine routine) -> {
      autoChooser.addRoutine(name, () -> routine);
    });

    SmartDashboard.putData("Auto chooser", autoChooser);
    SmartDashboard.putNumber("wait seconds", 0.0);
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
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
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
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

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

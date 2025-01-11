// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.util.Map;

import com.ctre.phoenix6.signals.NeutralModeValue;

import choreo.auto.AutoChooser;
import choreo.auto.AutoRoutine;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.CHOREO;
import frc.robot.Constants.CONTROLLER;
import frc.robot.commands.auto.Autos;
import frc.robot.commands.auto.CubeTestPath;
import frc.robot.commands.drive.ForceGyroZero;
import frc.robot.commands.drive.SetCoastOnDisable;
import frc.robot.commands.state.GetAlliance;
import frc.robot.controls.DriverControls;
import frc.robot.generated.TunerConstants;
import frc.robot.state.RobotState;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private static RobotContainer m_robotContainer;

  public static CommandSwerveDrivetrain swerve;
  public static RobotState state;
  public static DriverControls driverControls;
  public static AutoChooser autoChooser;

  private Autos m_autos;

  private Command m_setCoastOnDisable;
  private Command m_allianceGetter;
  private Command m_forceGyroZero;

  private Map<String, Command> m_autoCommandsToBind;
  private Map<String, AutoRoutine> m_autoRoutinesToBind;

  public Robot(){
    swerve = TunerConstants.createDrivetrain();
    state = new RobotState();
    driverControls = new DriverControls(new XboxController(CONTROLLER.DRIVE_CONTROLLER_PORT), CONTROLLER.DRIVE_CONTROLLER_DEADBAND);
    autoChooser = new AutoChooser();
    m_robotContainer = new RobotContainer();

    m_autos = new Autos();

    m_setCoastOnDisable = new SetCoastOnDisable();
    m_setCoastOnDisable.schedule();
    m_allianceGetter = new GetAlliance();
    m_allianceGetter.schedule();
    m_forceGyroZero = new ForceGyroZero();
    m_forceGyroZero.schedule();
    
    SendableBuilderImpl autoChooserBuilder = new SendableBuilderImpl();
    autoChooserBuilder.setTable(NetworkTableInstance.getDefault().getTable("SmartDashboard/Auto chooser"));
    autoChooser.initSendable(autoChooserBuilder);
    m_autoCommandsToBind = Map.of(
    
    );
    m_autoRoutinesToBind = Map.of(
      "CubeTestPath", m_autos.cubeTestPath()
    );

    m_autoCommandsToBind.forEach((String name, Command command) -> {autoChooser.addCmd(name, () -> command);});
    m_autoRoutinesToBind.forEach((String name, AutoRoutine routine) -> {autoChooser.addRoutine(name, () -> routine);});
    
    SmartDashboard.putData("Auto chooser", autoChooser);
    SmartDashboard.putNumber("wait seconds", 0.0);
    System.out.println("*******************EXISTS: " + SmartDashboard.containsKey("wait1234"));

    DriverStation.silenceJoystickConnectionWarning(true); // TODO: remove this if its a match
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_forceGyroZero.cancel();
    m_setCoastOnDisable.cancel();
    Robot.swerve.configNeutralMode(NeutralModeValue.Brake);
    m_autonomousCommand = autoChooser.selectedCommandScheduler();
    if(m_autonomousCommand != null){
      System.out.println("AUTO COMMAND NAME:" + m_autonomousCommand.getName());
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
     m_forceGyroZero.cancel();

    // Robot.swerve.setGyroOffset();
    m_setCoastOnDisable.cancel(); // very important, no touchy
    Robot.swerve.configNeutralMode(NeutralModeValue.Brake); // see above
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}

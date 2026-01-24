// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.AlgaePivotTuningSubsystem;
import frc.robot.subsystems.ElevatorTuningSubsystem;
import frc.robot.subsystems.LateratorTuningSubsystem;

public class Robot extends TimedRobot {

  ElevatorTuningSubsystem elevator;
  AlgaePivotTuningSubsystem algae;
  LateratorTuningSubsystem laterator;

  XboxController m_controller;

  public Robot() {
    m_controller = new XboxController(0);

    // elevator = new ElevatorTuningSubsystem();
    // algae = new AlgaePivotTuningSubsystem();
    laterator = new LateratorTuningSubsystem();
  }

  @Override
  public void robotPeriodic() {
    // elevator.updateDashboard();
    laterator.updateDashboard();
    // algae.updateDashboard();
  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    // elevator.teleopPeriodic();
    laterator.teleopPeriodic();
    // algae.teleopPeriodic();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {
    if (m_controller.getXButton()) {
      // elevator.setZero();
      laterator.setZero();
      // algae.setZero();
    }

    // elevator.setAxisSpeed(-m_controller.getRightY());
    laterator.setAxisSpeed(m_controller.getRightX());
    // algae.setAxisSpeed(-m_controller.getRightY());
    // Call elevator.setAxisSpeed with controller right joystick y axis value

  }
}

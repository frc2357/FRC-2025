package frc.robot.util;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;
import frc.robot.Robot;

public class ElasticFieldManager {

  private Field2d m_swerveFieldRep;
  private Field2d m_shooterFieldRep;

  /**
   * Contains all the logic and variables to put fields on elastic to look at the pose of them.
   */
  public ElasticFieldManager() {}

  private void setupSwerveField() {
    var swervefieldHelper = new SendableBuilderImpl();
    swervefieldHelper.setTable(
      NetworkTableInstance.getDefault().getTable("SmartDashboard/SwerveField")
    );
    m_swerveFieldRep = new Field2d();
    m_swerveFieldRep.initSendable(swervefieldHelper);
    m_swerveFieldRep.setRobotPose(Robot.swerve.getFieldRelativePose2d());
    swervefieldHelper.close();
  }

  private void setupShooterField() {
    var shooterFieldHelper = new SendableBuilderImpl();
    shooterFieldHelper.setTable(
      NetworkTableInstance.getDefault().getTable("SmartDashboard/ShooterField")
    );
    m_shooterFieldRep = new Field2d();
    m_shooterFieldRep.initSendable(shooterFieldHelper);
    m_shooterFieldRep.setRobotPose(Robot.swerve.getFieldRelativePose2d());
    shooterFieldHelper.close();
  }

  public Field2d getShooterField() {
    return m_shooterFieldRep;
  }

  public Field2d getSwerveField() {
    return m_swerveFieldRep;
  }
}

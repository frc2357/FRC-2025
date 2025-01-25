package frc.robot.util;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;
import frc.robot.Robot;

public class ElasticFieldManager {

  public Field2d swerveFieldRep;
  public Field2d shooterFieldRep;

  /**
   * Contains all the logic and variables to put fields on elastic to look at the pose of them.
   */
  public ElasticFieldManager() {}

  public void setupSwerveField() {
    var swervefieldHelper = new SendableBuilderImpl();
    swervefieldHelper.setTable(
      NetworkTableInstance.getDefault().getTable("SmartDashboard/SwerveField")
    );
    swerveFieldRep = new Field2d();
    swerveFieldRep.initSendable(swervefieldHelper);
    swerveFieldRep.setRobotPose(Robot.swerve.getFieldRelativePose2d());
    swervefieldHelper.close();
  }

  public void setupShooterField() {
    var shooterFieldHelper = new SendableBuilderImpl();
    shooterFieldHelper.setTable(
      NetworkTableInstance.getDefault().getTable("SmartDashboard/ShooterField")
    );
    shooterFieldRep = new Field2d();
    shooterFieldRep.initSendable(shooterFieldHelper);
    shooterFieldRep.setRobotPose(Robot.swerve.getFieldRelativePose2d());
    shooterFieldHelper.close();
  }
}

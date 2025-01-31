import edu.wpi.first.wpilibj.Timer;

public Timer timer = new Timer();

public void autonomousInit() {
    var swerve = {};
    timer.start();
    swerve.setControl(
        new SwerveRequest.FieldCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage).withVelocityX(-1).withVelocityY(0).withRotationalRate(0);
    );
}

public void autonomousPeriodic() {
    if (timer.hasElapsed(3)) {
        var swerve = {};
        swerve.setControl(
            new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage).withVelocityX(-1).withVelocityY(0).withRotationalRate(0);
        );
        timer.reset();
        timer.stop();
    }
}
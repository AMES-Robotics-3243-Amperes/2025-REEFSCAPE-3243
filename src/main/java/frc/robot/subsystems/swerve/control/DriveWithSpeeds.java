package frc.robot.subsystems.swerve.control;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.DataManager;

public class DriveWithSpeeds extends SwerveControlRequest {
  ChassisSpeeds speeds;
  private boolean fieldRelative = true;

  private DriveWithSpeeds(ChassisSpeeds speeds) {
    this.speeds = speeds;
  }

  public static DriveWithSpeeds newRequest(ChassisSpeeds speeds) {
    return new DriveWithSpeeds(speeds);
  }

  public static DriveWithSpeeds newRequest(Translation2d speeds, double rotationSpeed) {
    return new DriveWithSpeeds(new ChassisSpeeds(speeds.getX(), speeds.getY(), rotationSpeed));
  }

  public static DriveWithSpeeds newRequest(double xSpeed, double ySpeed, double rotationSpeed) {
    return new DriveWithSpeeds(new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed));
  }

  public static DriveWithSpeeds newStopDrivetrainRequest() {
    return new DriveWithSpeeds(new ChassisSpeeds(0, 0, 0));
  }

  public DriveWithSpeeds limitSpeed(boolean limitSpeed) {
    this.limitSpeed = limitSpeed;
    return this;
  }

  public DriveWithSpeeds limitAccelaration(boolean limitAccelaration) {
    this.limitAccelaration = limitAccelaration;
    return this;
  }

  public DriveWithSpeeds fieldRelative(boolean fieldRelative) {
    this.fieldRelative = fieldRelative;
    return this;
  }

  @Override
  protected ControlOutput getRawOutput(Translation2d currentFieldOrientedVelocity, double secondsSinceLastControl) {
    return fieldRelative
        ? new ControlOutput(
            ChassisSpeeds.fromFieldRelativeSpeeds(speeds, DataManager.instance().robotPosition.get().getRotation()))
        : new ControlOutput(speeds);
  }
}

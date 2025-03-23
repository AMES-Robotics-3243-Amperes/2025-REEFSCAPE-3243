package frc.robot.subsystems.swerve.control;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.DataManager;
import frc.robot.Constants.ElevatorPositions;
import frc.robot.Constants.SwerveConstants.ChassisKinematics;
import frc.robot.Constants.SwerveConstants.ControlConstants;

public abstract class SwerveControlRequest {
  public static class ControlOutput {
    private Optional<SwerveModuleState[]> states = Optional.empty();
    private Optional<ChassisSpeeds> robotRelativeSpeeds = Optional.empty();

    public ControlOutput(SwerveModuleState[] states) {
      this.states = Optional.of(states);
    }

    public ControlOutput(ChassisSpeeds robotRelativeSpeeds) {
      this.robotRelativeSpeeds = Optional.of(robotRelativeSpeeds);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
      if (!robotRelativeSpeeds.isPresent()) {
        robotRelativeSpeeds = Optional.of(ChassisKinematics.kDriveKinematics.toChassisSpeeds(states.get()));
      }

      return robotRelativeSpeeds.get();
    }

    public SwerveModuleState[] getStates() {
      if (!states.isPresent()) {
        states = Optional.of(ChassisKinematics.kDriveKinematics.toSwerveModuleStates(robotRelativeSpeeds.get()));
      }

      return states.get();
    }
  }

  protected boolean limitAccelaration = false;
  protected boolean limitSpeed = false;

  protected abstract ControlOutput getRawOutput(Translation2d currentFieldOrientedVelocity,
      double secondsSinceLastControl);

  public final ControlOutput getOutput(Translation2d currentFieldOrientedVelocity, double secondsSinceLastControl) {
    ControlOutput rawOutput = getRawOutput(currentFieldOrientedVelocity, secondsSinceLastControl);
    if (!limitAccelaration && !limitSpeed) {
      return rawOutput;
    }

    Translation2d robotRelativeVelocity = new Translation2d(rawOutput.getRobotRelativeSpeeds().vxMetersPerSecond,
        rawOutput.getRobotRelativeSpeeds().vyMetersPerSecond);

    // speed limiting
    if (limitSpeed) {
      double maxSpeed = MathUtil.interpolate(ControlConstants.maxSpeed, ControlConstants.maxSpeedAtMaxElevatorExtension,
          DataManager.instance().elevatorPosition.get().exactPos / ElevatorPositions.max);
      double currentSpeed = robotRelativeVelocity.getNorm();

      if (currentSpeed > maxSpeed) {
        robotRelativeVelocity = robotRelativeVelocity.times(maxSpeed / currentSpeed);
      }
    }

    // acceleration limiting
    if (limitAccelaration) {
      Rotation2d robotRotation = DataManager.instance().robotPosition.get().getRotation();

      Translation2d fieldOrientedVelocity = robotRelativeVelocity.rotateBy(robotRotation);
      Translation2d acceleration = fieldOrientedVelocity.minus(currentFieldOrientedVelocity);

      // according to hale, this is the right way to compute this
      double maxAcceleration = secondsSinceLastControl *
          (ControlConstants.baseAcceleration * ControlConstants.baseCenterOfMass / ControlConstants.accelerationLimitSafetyFactor)
          / (ControlConstants.baseCenterOfMass
              + ControlConstants.percentOfWeightInElevator * DataManager.instance().elevatorPosition.get().exactPos);
      double accelerationNorm = acceleration.getNorm();

      if (accelerationNorm > maxAcceleration) {
        acceleration = acceleration.times(maxAcceleration / accelerationNorm);
      }

      fieldOrientedVelocity = currentFieldOrientedVelocity.plus(acceleration);
      robotRelativeVelocity = fieldOrientedVelocity.rotateBy(robotRotation.times(-1));
    }

    return new ControlOutput(new ChassisSpeeds(robotRelativeVelocity.getX(), robotRelativeVelocity.getY(),
        rawOutput.getRobotRelativeSpeeds().omegaRadiansPerSecond));
  }
}
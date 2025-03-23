// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.JoyUtil;
import frc.robot.Constants.ElevatorPositions;
import frc.robot.Constants.SwerveConstants.ChassisKinematics;
import frc.robot.Constants.SwerveConstants.ControlConstants;
import frc.robot.DataManager;
import frc.robot.subsystems.SubsystemSwerveDrivetrain;

public class CommandSwerveTeleopDrive extends Command {
  // :3 subsystem
  private final SubsystemSwerveDrivetrain subsystemSwerveDrivetrain;

  // :3 driver joyutil
  private final JoyUtil controller;

  private boolean fieldRelative = true;
  private boolean redAlliance = false;

  private Translation2d velocity = new Translation2d();
  private Timer accelerationTimer = new Timer();

  /**
   * Creates a new SwerveTeleopCommand.
   * 
   * @author :3
   */
  public CommandSwerveTeleopDrive(SubsystemSwerveDrivetrain subsystem, JoyUtil controller) {
    subsystemSwerveDrivetrain = subsystem;
    this.controller = controller;

    addRequirements(subsystem);
  }

  public void toggleFieldRelative() {
    this.fieldRelative = !this.fieldRelative;
  }

  @Override
  public void initialize() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      redAlliance = (alliance.get() == Alliance.Red);
    }

    velocity = new Translation2d();
    accelerationTimer.restart();
  }

  @Override
  public void execute() {
    Translation2d rawControllerVelocity = new Translation2d(controller.getLeftY(), controller.getLeftX());
    Translation2d desiredVelocity = rawControllerVelocity.times(ControlConstants.movingSpeed);

    desiredVelocity = desiredVelocity
        .times(MathUtil.interpolate(1, ControlConstants.leftTriggerMultiplier, controller.getLeftTriggerAxis()))
        .times(MathUtil.interpolate(1, ControlConstants.rightTriggerMultiplier, controller.getRightTriggerAxis()));

    double desiredSpeed = desiredVelocity.getNorm();
    double maxSpeed = MathUtil.interpolate(ControlConstants.maxSpeed,
        ControlConstants.maxSpeedAtMaxElevatorExtension,
        DataManager.instance().elevatorPosition.get().exactPos / ElevatorPositions.max);
    if (desiredSpeed > maxSpeed) {
      desiredVelocity = desiredVelocity.div(desiredSpeed).times(maxSpeed);
    }

    // convert to field-relative control for acceleration capping
    if (!fieldRelative) {
      desiredVelocity = desiredVelocity.rotateBy(DataManager.instance().robotPosition.get().getRotation());
    } else if (redAlliance) {
      desiredVelocity = desiredVelocity.rotateBy(Rotation2d.fromDegrees(180));
    }

    Translation2d acceleration = desiredVelocity.minus(velocity);
    double desiredAccelerationNorm = acceleration.getNorm();
    double maxAcceleration = (accelerationTimer.get() * ControlConstants.baseAcceleration
        * ControlConstants.baseCenterOfMass)
        / (ControlConstants.baseCenterOfMass
            + ControlConstants.percentOfWeightInElevator * DataManager.instance().elevatorPosition.get().exactPos);
    if (desiredAccelerationNorm > maxAcceleration) {
      acceleration = acceleration.div(desiredAccelerationNorm).times(maxAcceleration);
    }

    velocity = velocity.plus(acceleration);
    accelerationTimer.restart();

    double rotationSpeed = -controller.getRightX() * ControlConstants.rotationSpeed;

    Translation2d robotRelativeVelocity = velocity
        .rotateBy(DataManager.instance().robotPosition.get().getRotation().times(-1));

    // :3 drive with those speeds
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(robotRelativeVelocity.getX(), robotRelativeVelocity.getY(),
        rotationSpeed);
    SwerveModuleState[] moduleStates = ChassisKinematics.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    subsystemSwerveDrivetrain.setModuleStates(moduleStates);
  }

  @Override
  public void end(boolean interrupted) {
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
    SwerveModuleState[] moduleStates = ChassisKinematics.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    subsystemSwerveDrivetrain.setModuleStates(moduleStates);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
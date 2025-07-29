// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.JoyUtil;
import frc.robot.Constants.SwerveConstants.ControlConstants;
import frc.robot.subsystems.swerve.SubsystemSwerveDrivetrain;
import frc.robot.subsystems.swerve.control.DriveWithSpeeds;

public class CommandSwerveTeleopDrive extends Command {
  // :3 subsystem
  private final SubsystemSwerveDrivetrain drivetrain;

  // :3 driver joyutil
  private final JoyUtil controller;

  private boolean fieldRelative = true;
  private boolean redAlliance = false;

  /**
   * Creates a new SwerveTeleopCommand.
   * 
   * @author :3
   */
  public CommandSwerveTeleopDrive(SubsystemSwerveDrivetrain subsystem, JoyUtil controller) {
    drivetrain = subsystem;
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
  }

  @Override
  public void execute() {
    Translation2d rawControllerVelocity = new Translation2d(-controller.getLeftY(), -controller.getLeftX());
    Translation2d velocity = rawControllerVelocity.times(ControlConstants.movingSpeed);
    velocity = velocity
        .times(MathUtil.interpolate(1, ControlConstants.leftTriggerMultiplier, controller.getLeftTriggerAxis()))
        .times(MathUtil.interpolate(1, ControlConstants.rightTriggerMultiplier, controller.getRightTriggerAxis()));
    
    if (fieldRelative && redAlliance) {
      velocity = velocity.unaryMinus();
    }

    double rotationSpeed = -controller.getRightX() * ControlConstants.rotationSpeed;

    DriveWithSpeeds request = DriveWithSpeeds.newRequest(velocity, rotationSpeed)
        .fieldRelative(fieldRelative)
        .limitSpeed(true).limitAccelaration(true);
    drivetrain.setControlRequest(request);
  }


  @Override
  public void end(boolean interrupted) {
    drivetrain
        .setControlRequest(DriveWithSpeeds.newStopDrivetrainRequest().limitAccelaration(true));
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
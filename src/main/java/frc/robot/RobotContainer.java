// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.CommandSwerveTeleopDrive;
import frc.robot.commands.CommandSwerveXWheels;
import frc.robot.commands.GetCameraOffset;
import frc.robot.commands.automatics.L1AutoCommand;
import frc.robot.commands.elevator.ElevatorMoveToPositionCommand;
import frc.robot.commands.elevator.ElevatorNudgeCommand;
import frc.robot.commands.elevator.ElevatorZeroCommand;
import frc.robot.commands.leds.CommandLedPatternCycle;
import frc.robot.splines.PathFactory;
import frc.robot.subsystems.SubsystemElevator;
import frc.robot.subsystems.SubsystemEndEffector;
import frc.robot.subsystems.SubsystemLeds;
import frc.robot.subsystems.SubsystemSwerveDrivetrain;
import frc.robot.Constants.Elevator;
import frc.robot.Constants.ElevatorPositions;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SplineConstants.FollowConstants;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // H! Main shuffleboard tab
  ShuffleboardTab mainTab = Shuffleboard.getTab("Main");

  // H! Auto Selector
  AutoSelector autoSelector = new AutoSelector(mainTab);

  // controllers
  private JoyUtil primaryController = new JoyUtil(0);
  private JoyUtil secondaryController = new JoyUtil(1);

  // Path Factory for auto routine
  // PathFactory pathFactory = PathFactory.newFactory();

  //
  // Subsystems
  //

  public SubsystemSwerveDrivetrain subsystemSwerveDrivetrain = new SubsystemSwerveDrivetrain();
  public SubsystemLeds subsystemLeds = new SubsystemLeds();
  public SubsystemElevator subsystemElevator = new SubsystemElevator();
  public SubsystemEndEffector endEffector = new SubsystemEndEffector();

  //
  // Commands
  //

  private CommandSwerveTeleopDrive commandSwerveTeleopDrive = new CommandSwerveTeleopDrive(subsystemSwerveDrivetrain,
      primaryController);
  private CommandLedPatternCycle commandLedPatternCycle = new CommandLedPatternCycle(subsystemLeds);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // we construct the DataManager instance here since it is the
    // absolute soonest we have access to a RobotContainer object
    new DataManager(this);

    // set sensible default commands
    setDefaultCommands();
    mainTab.add(subsystemSwerveDrivetrain);

    // H! Set all commands in auto selector
    setAutoCommands();

    // configure the controller bindings
    configureBindings();
  }

  /**
   * Used to set default commands for subsystems.
   */
  private void setDefaultCommands() {
    subsystemLeds.setDefaultCommand(commandLedPatternCycle);
    subsystemSwerveDrivetrain.setDefaultCommand(commandSwerveTeleopDrive);
  }

  private void setAutoCommands() {
  }

  /**
   * Used to configure controller bindings.
   * Do not remove any of the commented out code. Most of it is commented for
   * testing purposes.
   */
  private void configureBindings() {
    secondaryController.a().onTrue(
        new ElevatorMoveToPositionCommand(subsystemElevator, ElevatorPositions.L1));

    secondaryController.x().onTrue(
        new ElevatorMoveToPositionCommand(subsystemElevator, ElevatorPositions.L2));

    secondaryController.y().onTrue(
        new ElevatorMoveToPositionCommand(subsystemElevator, ElevatorPositions.L3));

    secondaryController.b().onTrue(
        new ElevatorMoveToPositionCommand(subsystemElevator, ElevatorPositions.L4));

    secondaryController.povRight().onTrue(
        new ElevatorMoveToPositionCommand(subsystemElevator, ElevatorPositions.loading));

    // Manual intaking/depositing, elevator movement, reef setpoints
    secondaryController.leftBumper().whileTrue(endEffector.intakeCommand());
    secondaryController.rightBumper().whileTrue(endEffector.continuousOuttakeCommand());

    Trigger leftYUp = new Trigger(() -> secondaryController.getLeftY() < -Elevator.manualThreshold);
    Trigger leftYDown = new Trigger(() -> secondaryController.getLeftY() > Elevator.manualThreshold);

    leftYUp.whileTrue(new ElevatorNudgeCommand(subsystemElevator, Constants.Elevator.Control.upNudgeVelocity));
    leftYDown.whileTrue(new ElevatorNudgeCommand(subsystemElevator, Constants.Elevator.Control.downNudgeVelocity));

    mainTab.add("Zero Elevator", new ElevatorZeroCommand(subsystemElevator)).withWidget(BuiltInWidgets.kCommand);
    secondaryController.povUp()
        .onTrue(
            new GetCameraOffset(new PhotonCamera("FrontLeftCamera"),
                new Transform3d(new Pose3d(), new Pose3d(Units.inchesToMeters(6.95 + 13), 0, Units.inchesToMeters(11.8),
                    new Rotation3d(Rotation2d.fromDegrees(180))))));
    // secondaryController.povDown().onTrue(newCommandSwerveGetOffset(subsystemSwerveDrivetrain));
    secondaryController.povDown().whileTrue(PathFactory.newFactory().addPoint(FieldConstants.fieldLayout.getTagPose(9)
        .get().plus(new Transform3d(Units.inchesToMeters(13 + 4), -Units.inchesToMeters(12.9 / 2.0) * 0, 0, new Rotation3d())).getTranslation().toTranslation2d())
        .interpolateFromStart(true).buildCommand(subsystemSwerveDrivetrain, FollowConstants.xyController(),
            FollowConstants.xyController(), FollowConstants.thetaController()));

    primaryController.b().onTrue(Commands.runOnce(commandSwerveTeleopDrive::toggleFieldRelative));
    primaryController.a().whileTrue(new CommandSwerveXWheels(subsystemSwerveDrivetrain));
    primaryController.leftBumper().onTrue(new ElevatorZeroCommand(subsystemElevator));

    primaryController.x().or(primaryController.rightBumper())
        .onTrue(new ElevatorMoveToPositionCommand(subsystemElevator, ElevatorPositions.store));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new L1AutoCommand(subsystemSwerveDrivetrain, endEffector);
    // return ScoreIntakeAutoCommandBuilder.buildAuto(
    // FieldConstants.AutonomousPaths.testingPositions,
    // subsystemClaw, subsystemElevator,
    // subsystemSwerveDrivetrain, FieldConstants.AutonomousPaths.testingSetpoints);
    // return new SequentialCommandGroup(new
    // CommandSwerveTaxi(subsystemSwerveDrivetrain),
    // ScoreIntakeAutoCommandBuilder.buildAuto(
    // FieldConstants.AutonomousPaths.testingPositions,
    // subsystemClaw, subsystemElevator,
    // subsystemSwerveDrivetrain, FieldConstants.AutonomousPaths.testingSetpoints));
  }
}

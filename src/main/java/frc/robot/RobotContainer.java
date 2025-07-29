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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.CommandSwerveGetOffset;
import frc.robot.commands.CommandSwerveTeleopDrive;
import frc.robot.commands.CommandSwerveXWheels;
import frc.robot.commands.GetCameraOffset;
import frc.robot.commands.automatics.L1AutoCommand;
import frc.robot.commands.automatics.L1DoubleHitScore;
import frc.robot.commands.automatics.L2AutoCommand;
import frc.robot.commands.automatics.L4AutoCommand;
import frc.robot.commands.automatics.PositionUtils;
import frc.robot.commands.automatics.TaxiCommand;
import frc.robot.commands.elevator.ElevatorMoveToPositionCommand;
import frc.robot.commands.elevator.ElevatorNudgeCommand;
import frc.robot.commands.elevator.ElevatorZeroCommand;
import frc.robot.commands.leds.CommandLedPattern;
import frc.robot.commands.leds.CommandLedsFromElevatorPosition;
import frc.robot.subsystems.SubsystemElevator;
import frc.robot.subsystems.SubsystemEndEffector;
import frc.robot.subsystems.SubsystemLeds;
import frc.robot.subsystems.SubsystemLeds.Mode;
import frc.robot.subsystems.swerve.SubsystemSwerveDrivetrain;
import frc.robot.Constants.Elevator;
import frc.robot.Constants.ElevatorPositions;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import java.util.function.BooleanSupplier;

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

      
  public enum ControlScheme { MULTIPLAYER, SINGLEPLAYER }
  private final SendableChooser<ControlScheme> controlSchemeChooser = new SendableChooser<>();

  private final BooleanSupplier isSingleplayer = () -> controlSchemeChooser.getSelected() == ControlScheme.SINGLEPLAYER;
  private final BooleanSupplier isMultiplayer  = () -> controlSchemeChooser.getSelected() != ControlScheme.SINGLEPLAYER;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // we construct the DataManager instance here since it is the
    // absolute soonest we have access to a RobotContainer object
    new DataManager(this);
    controlSchemeChooser.setDefaultOption("Multiplayer", ControlScheme.MULTIPLAYER);
    controlSchemeChooser.addOption("Singleplayer", ControlScheme.SINGLEPLAYER);
    mainTab.add("Control Scheme", controlSchemeChooser)
       .withWidget(BuiltInWidgets.kComboBoxChooser);

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
    subsystemLeds.setDefaultCommand
    (
        new SequentialCommandGroup
        (
            new CommandLedsFromElevatorPosition(subsystemLeds, DataManager.instance()).withTimeout(10),
            new CommandLedPattern(subsystemLeds, Mode.OrangeFire).withTimeout(10),
            new CommandLedPattern(subsystemLeds, Mode.HueCircle).withTimeout(10),
            new CommandLedPattern(subsystemLeds, Mode.TransFlag).withTimeout(10)
        ).repeatedly()
    );
    subsystemSwerveDrivetrain.setDefaultCommand(commandSwerveTeleopDrive);
  }

  private void setAutoCommands() {
    autoSelector.addDefault(new InstantCommand(), "None");
    autoSelector.add(new TaxiCommand(subsystemSwerveDrivetrain, 0.5), "Small Taxi");
    autoSelector.add(new L1AutoCommand(subsystemSwerveDrivetrain, subsystemElevator, endEffector, DataManager.instance()));
    autoSelector.add(new L2AutoCommand(subsystemSwerveDrivetrain, subsystemElevator, endEffector));
    autoSelector.add(new L4AutoCommand(subsystemSwerveDrivetrain, subsystemElevator, endEffector));
  }

  /**
   * Used to configure controller bindings.
   * Do not remove any of the commented out code. Most of it is commented for
   * testing purposes.
   */
  private void configureBindings() {
    Trigger multiplayer  = new Trigger(isMultiplayer);
    Trigger single = new Trigger(isSingleplayer);

    multiplayer.and(secondaryController.a()).onTrue(
        new ElevatorMoveToPositionCommand(subsystemElevator, ElevatorPositions.L1));

    multiplayer.and(secondaryController.x()).onTrue(
        new ElevatorMoveToPositionCommand(subsystemElevator, ElevatorPositions.L2));

    multiplayer.and(secondaryController.y()).onTrue(
        new ElevatorMoveToPositionCommand(subsystemElevator, ElevatorPositions.L3));

    multiplayer.and(secondaryController.b()).onTrue(
        new ElevatorMoveToPositionCommand(subsystemElevator, ElevatorPositions.L4));

    multiplayer.and(secondaryController.povRight()).onTrue(
        new ElevatorMoveToPositionCommand(subsystemElevator, ElevatorPositions.loading));

    multiplayer.and(secondaryController.start()).whileTrue(
        new L1DoubleHitScore(subsystemElevator, endEffector, DataManager.instance()));

    // Manual intaking/depositing, elevator movement, reef setpoints
    multiplayer.and(secondaryController.leftBumper()).whileTrue(endEffector.intakeCommand());
    multiplayer.and(secondaryController.rightBumper()).whileTrue(endEffector.continuousOuttakeCommand());

    Trigger leftYUp   = new Trigger(() -> isMultiplayer.getAsBoolean() && (secondaryController.getLeftY() < -Elevator.manualThreshold));
    Trigger leftYDown = new Trigger(() -> isMultiplayer.getAsBoolean() && (secondaryController.getLeftY() >  Elevator.manualThreshold));
    

    leftYUp.whileTrue(new ElevatorNudgeCommand(subsystemElevator, Constants.Elevator.Control.upNudgeVelocity));
    leftYDown.whileTrue(new ElevatorNudgeCommand(subsystemElevator, Constants.Elevator.Control.downNudgeVelocity));

    mainTab.add("Zero Elevator", new ElevatorZeroCommand(subsystemElevator)).withWidget(BuiltInWidgets.kCommand);

    multiplayer.and(primaryController.b()).onTrue(Commands.runOnce(commandSwerveTeleopDrive::toggleFieldRelative));
    multiplayer.and(primaryController.a()).whileTrue(new CommandSwerveXWheels(subsystemSwerveDrivetrain));
    multiplayer.and(primaryController.leftBumper()).onTrue(new ElevatorZeroCommand(subsystemElevator));

    multiplayer.and(primaryController.x().or(primaryController.rightBumper()))
        .onTrue(new ElevatorMoveToPositionCommand(subsystemElevator, ElevatorPositions.store));
    
    // reef automatics
    multiplayer.and(primaryController.povUpRight().and(primaryController.y())).onTrue(
        PositionUtils.moveToNearestScorePositionCommand(false, ElevatorPositions.L4,
            subsystemSwerveDrivetrain, subsystemElevator)
    );
    multiplayer.and(primaryController.povRight().and(primaryController.y())).onTrue(
        PositionUtils.moveToNearestScorePositionCommand(false, ElevatorPositions.L3,
            subsystemSwerveDrivetrain, subsystemElevator)
    );
    multiplayer.and(primaryController.povDownRight().and(primaryController.y())).onTrue(
        PositionUtils.moveToNearestScorePositionCommand(false, ElevatorPositions.L2,
            subsystemSwerveDrivetrain, subsystemElevator)
    );
    multiplayer.and(primaryController.povDownLeft().and(primaryController.y())).onTrue(
        PositionUtils.moveToNearestScorePositionCommand(true, ElevatorPositions.L2,
            subsystemSwerveDrivetrain, subsystemElevator)
    );
    multiplayer.and(primaryController.povLeft().and(primaryController.y())).onTrue(
        PositionUtils.moveToNearestScorePositionCommand(true, ElevatorPositions.L3,
            subsystemSwerveDrivetrain, subsystemElevator)
    );
    multiplayer.and(primaryController.povUpLeft().and(primaryController.y())).onTrue(
        PositionUtils.moveToNearestScorePositionCommand(true, ElevatorPositions.L4,
            subsystemSwerveDrivetrain, subsystemElevator)
    );
    
    // debug info
    multiplayer.and(secondaryController.povUp())
        .onTrue(
            new GetCameraOffset(new PhotonCamera("FrontLeftCamera"),
                new Transform3d(new Pose3d(),
                    new Pose3d(1, 0, Units.inchesToMeters(11.8),
                        new Rotation3d(Rotation2d.fromDegrees(180))))));
    multiplayer.and(secondaryController.povDown()).onTrue(new CommandSwerveGetOffset(subsystemSwerveDrivetrain));
    
        // SINGLEPLAYER 
    Trigger lTrigHeld = new Trigger(() -> isSingleplayer.getAsBoolean() && primaryController.getLeftTriggerAxis()  > 0.5);
    Trigger rTrigHeld = new Trigger(() -> isSingleplayer.getAsBoolean() && primaryController.getRightTriggerAxis() > 0.5);
    lTrigHeld.whileTrue(endEffector.intakeCommand());
    rTrigHeld.whileTrue(endEffector.continuousOuttakeCommand());

    single.and(primaryController.leftBumper()).onTrue(new ElevatorZeroCommand(subsystemElevator));

    single.and(primaryController.start()).onTrue(Commands.runOnce(commandSwerveTeleopDrive::toggleFieldRelative));

    single.and(primaryController.a()).onTrue(new ElevatorMoveToPositionCommand(subsystemElevator, ElevatorPositions.store));
    single.and(primaryController.b()).onTrue(new ElevatorMoveToPositionCommand(subsystemElevator, ElevatorPositions.L2));
    single.and(primaryController.x()).onTrue(new ElevatorMoveToPositionCommand(subsystemElevator, ElevatorPositions.L3));
    single.and(primaryController.y()).onTrue(new ElevatorMoveToPositionCommand(subsystemElevator, ElevatorPositions.L4));

    single.and(primaryController.povUp())
        .whileTrue(new ElevatorNudgeCommand(subsystemElevator, Constants.Elevator.Control.upNudgeVelocity));
    single.and(primaryController.povDown())
        .whileTrue(new ElevatorNudgeCommand(subsystemElevator, Constants.Elevator.Control.downNudgeVelocity));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoSelector.get();
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

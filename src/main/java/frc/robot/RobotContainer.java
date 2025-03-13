// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.SplineConstants.FollowConstants;
import frc.robot.DataManager.Setpoint;
import frc.robot.commands.CommandClimber;
import frc.robot.commands.CommandSwerveFollowSpline;
import frc.robot.commands.CommandSwerveTeleopDrive;
import frc.robot.commands.CommandSwerveXWheels;
import frc.robot.commands.automatics.ScoreIntakeAutoCommandBuilder;
import frc.robot.commands.claw.DeployClawCommand;
import frc.robot.commands.claw.IntakeClawCommand;
import frc.robot.commands.elevator.ElevatorMoveToPositionCommand;
import frc.robot.commands.elevator.ElevatorNudgeCommand;
import frc.robot.commands.elevator.ElevatorZeroCommand;
import frc.robot.commands.leds.CommandLedPatternCycle;
import frc.robot.splines.PathFactory;
import frc.robot.subsystems.SubsystemClaw;
import frc.robot.subsystems.SubsystemClimber;
import frc.robot.subsystems.SubsystemElevator;
import frc.robot.subsystems.SubsystemLeds;
import frc.robot.subsystems.SubsystemSwerveDrivetrain;
import frc.robot.Constants.Setpoints;
import frc.robot.Constants.Setpoints.LevelAngles;
import frc.robot.Constants.DifferentialArm;
import frc.robot.Constants.Elevator;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.Positions;

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
  PathFactory pathFactory = PathFactory.newFactory();

  //
  // Subsystems
  //

  public SubsystemSwerveDrivetrain subsystemSwerveDrivetrain = new SubsystemSwerveDrivetrain();
  public SubsystemLeds subsystemLeds = new SubsystemLeds();
  public SubsystemElevator subsystemElevator = new SubsystemElevator();
  public SubsystemClaw subsystemClaw = new SubsystemClaw();
  public SubsystemClimber subsystemClimber = new SubsystemClimber();

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

    mainTab.add(subsystemClaw);
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
    subsystemClimber.setDefaultCommand(new CommandClimber(secondaryController, subsystemClimber));
  }

  private void setAutoCommands() {
    autoSelector
        .add(
            ScoreIntakeAutoCommandBuilder.buildAuto(
                FieldConstants.AutonomousPaths.blueTopToIntakePositions,
                pathFactory, subsystemClaw, subsystemElevator,
                subsystemSwerveDrivetrain, FieldConstants.AutonomousPaths.intakeScoreBackAndForthSetpoints),
            "Blue Top")
        .add(
          ScoreIntakeAutoCommandBuilder.buildAuto(
              FieldConstants.AutonomousPaths.blueMiddleToTopIntakePositions,
              pathFactory, subsystemClaw, subsystemElevator,
              subsystemSwerveDrivetrain, FieldConstants.AutonomousPaths.intakeScoreBackAndForthSetpoints),
          "Blue Middle -> Top")
        .add(
          ScoreIntakeAutoCommandBuilder.buildAuto(
              FieldConstants.AutonomousPaths.blueMiddleToBottomIntakePositions,
              pathFactory, subsystemClaw, subsystemElevator,
              subsystemSwerveDrivetrain, FieldConstants.AutonomousPaths.intakeScoreBackAndForthSetpoints),
          "Blue Middle -> Bottom")
        .add(
          ScoreIntakeAutoCommandBuilder.buildAuto(
              FieldConstants.AutonomousPaths.blueBottomToIntakePositions,
              pathFactory, subsystemClaw, subsystemElevator,
              subsystemSwerveDrivetrain, FieldConstants.AutonomousPaths.intakeScoreBackAndForthSetpoints),
          "Blue Bottom")
        .add(
          ScoreIntakeAutoCommandBuilder.buildAuto(
              FieldConstants.AutonomousPaths.redTopToIntakePositions,
              pathFactory, subsystemClaw, subsystemElevator,
              subsystemSwerveDrivetrain, FieldConstants.AutonomousPaths.intakeScoreBackAndForthSetpoints),
          "Red Top")
        .add(
          ScoreIntakeAutoCommandBuilder.buildAuto(
              FieldConstants.AutonomousPaths.redMiddleToTopIntakePositions,
              pathFactory, subsystemClaw, subsystemElevator,
              subsystemSwerveDrivetrain, FieldConstants.AutonomousPaths.intakeScoreBackAndForthSetpoints),
          "Red Middle -> Top")
        .add(
          ScoreIntakeAutoCommandBuilder.buildAuto(
              FieldConstants.AutonomousPaths.redMiddleToBottomIntakePositions,
              pathFactory, subsystemClaw, subsystemElevator,
              subsystemSwerveDrivetrain, FieldConstants.AutonomousPaths.intakeScoreBackAndForthSetpoints),
          "Red Middle -> Bottom")
        .add(
          ScoreIntakeAutoCommandBuilder.buildAuto(
              FieldConstants.AutonomousPaths.blueBottomToIntakePositions,
              pathFactory, subsystemClaw, subsystemElevator,
              subsystemSwerveDrivetrain, FieldConstants.AutonomousPaths.intakeScoreBackAndForthSetpoints),
          "Red Bottom");
  }

  /**
   * Used to configure controller bindings.
   * Do not remove any of the commented out code. Most of it is commented for
   * testing purposes.
   */
  private void configureBindings() {
    secondaryController.a().onTrue(new ParallelCommandGroup(
      new ElevatorMoveToPositionCommand(subsystemElevator, Setpoint.L1Left.height),
      new InstantCommand(
          () -> {
            subsystemClaw.setOutsidePosition(Setpoint.L1Left.angle);
          },
          subsystemClaw)));

  secondaryController.x().onTrue(new ParallelCommandGroup(
      new ElevatorMoveToPositionCommand(subsystemElevator, Setpoint.L2Left.height),
      new InstantCommand(
          () -> {
            subsystemClaw.setOutsidePosition(Setpoint.L2Left.angle);
          },
          subsystemClaw)));

  secondaryController.y().onTrue(new ParallelCommandGroup(
      new ElevatorMoveToPositionCommand(subsystemElevator, Setpoint.L3Left.height),
      new InstantCommand(
          () -> {
            subsystemClaw.setOutsidePosition(Setpoint.L3Left.angle);
          },
          subsystemClaw)));

  secondaryController.b().onTrue(new ParallelCommandGroup(
      new ElevatorMoveToPositionCommand(subsystemElevator, Setpoint.L4Left.height),
      new InstantCommand(
          () -> {
            subsystemClaw.setOutsidePosition(Setpoint.L4Left.angle);
          },
          subsystemClaw)));

    // Auto loading
    double offsetInches = 7;
    secondaryController.povLeft().onTrue(
      ScoreIntakeAutoCommandBuilder.scoreIntakeAutoCommand(subsystemSwerveDrivetrain, subsystemClaw,
            subsystemElevator, Setpoint.IntakeLeft, -Units.inchesToMeters(offsetInches), false)
    );
    secondaryController.povRight().onTrue(new ParallelCommandGroup(
        new ElevatorMoveToPositionCommand(subsystemElevator, Setpoint.IntakeLeft.height),
        new InstantCommand(
            () -> {
              subsystemClaw.setOutsidePosition(Setpoint.IntakeLeft.angle);
            },
            subsystemClaw)));

    // Auto score in nearest L4 (Left or Right selected by D-pad)
    primaryController.y().and(primaryController.pov(225)).onTrue(
        ScoreIntakeAutoCommandBuilder.scoreIntakeAutoCommand(subsystemSwerveDrivetrain, subsystemClaw,
            subsystemElevator, Setpoint.L2Right, -Units.inchesToMeters(offsetInches), true));

    primaryController.y().and(primaryController.pov(135)).onTrue(
        ScoreIntakeAutoCommandBuilder.scoreIntakeAutoCommand(subsystemSwerveDrivetrain, subsystemClaw,
            subsystemElevator, Setpoint.L2Left, Units.inchesToMeters(offsetInches), true));

    primaryController.y().and(primaryController.pov(270)).onTrue(
        ScoreIntakeAutoCommandBuilder.scoreIntakeAutoCommand(subsystemSwerveDrivetrain, subsystemClaw,
            subsystemElevator, Setpoint.L3Right, -Units.inchesToMeters(offsetInches), true));

    primaryController.y().and(primaryController.pov(90)).onTrue(
        ScoreIntakeAutoCommandBuilder.scoreIntakeAutoCommand(subsystemSwerveDrivetrain, subsystemClaw,
            subsystemElevator, Setpoint.L3Left, Units.inchesToMeters(offsetInches), true));

    primaryController.y().and(primaryController.pov(315)).onTrue(
        ScoreIntakeAutoCommandBuilder.scoreIntakeAutoCommand(subsystemSwerveDrivetrain, subsystemClaw,
            subsystemElevator, Setpoint.L4Right, -Units.inchesToMeters(offsetInches), true));

    primaryController.y().and(primaryController.pov(45)).onTrue(
        ScoreIntakeAutoCommandBuilder.scoreIntakeAutoCommand(subsystemSwerveDrivetrain, subsystemClaw,
            subsystemElevator, Setpoint.L4Left, Units.inchesToMeters(offsetInches), true));

    // Manual intaking/depositing, elevator movement, reef setpoints
    secondaryController.leftBumper()
        .whileTrue(new IntakeClawCommand(subsystemClaw, frc.robot.Constants.Setpoints.intakePower));
    secondaryController.rightBumper()
        .whileTrue(new DeployClawCommand(subsystemClaw, -frc.robot.Constants.Setpoints.intakePower));

    Trigger leftYUp = new Trigger(() -> secondaryController.getLeftY() < -Elevator.manualThreshold);
    Trigger leftYDown = new Trigger(() -> secondaryController.getLeftY() > Elevator.manualThreshold);

    leftYUp.whileTrue(new ElevatorNudgeCommand(subsystemElevator, Constants.Elevator.Control.upNudgeVelocity));
    leftYDown.whileTrue(new ElevatorNudgeCommand(subsystemElevator, Constants.Elevator.Control.downNudgeVelocity));

    // Wrist pitch manual control
    Trigger rightYUp = new Trigger(() -> secondaryController.getRightY() < -DifferentialArm.manualThreshold);
    Trigger rightYDown = new Trigger(() -> secondaryController.getRightY() > DifferentialArm.manualThreshold);

    rightYUp.whileTrue(
        new RepeatCommand(
            new InstantCommand(
                () -> {
                  subsystemClaw.setOutsidePosition(
                      subsystemClaw.getPosition() +
                          DifferentialArm.manualMovementPerSecond / 50.0);
                },
                subsystemClaw)));

    rightYDown.whileTrue(
        new RepeatCommand(
            new InstantCommand(
                () -> {
                  subsystemClaw.setOutsidePosition(
                      subsystemClaw.getPosition() -
                          DifferentialArm.manualMovementPerSecond / 50.0);
                },
                subsystemClaw)));

    mainTab.add("Zero Elevator", new ElevatorZeroCommand(subsystemElevator)).withWidget(BuiltInWidgets.kCommand);

    // primaryController.x().toggleOnTrue(new
    // CommandSwerveGetOffset(subsystemSwerveDrivetrain));
    primaryController.b().onTrue(Commands.runOnce(commandSwerveTeleopDrive::toggleFieldRelative));
    primaryController.a().whileTrue(new CommandSwerveXWheels(subsystemSwerveDrivetrain));
    primaryController.leftBumper().onTrue(new ElevatorZeroCommand(subsystemElevator));

    primaryController.x().or(primaryController.rightBumper()).onTrue(new ParallelCommandGroup(
        new ElevatorMoveToPositionCommand(subsystemElevator, Setpoint.Store.height),
        new InstantCommand(
            () -> {
              subsystemClaw.setOutsidePosition(Setpoint.Store.angle);
            },
            subsystemClaw)));
  }

  public static double convertJoystickToPosition(double joystick) {
    return (DifferentialArm.encoderRange * joystick + DifferentialArm.encoderRange);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoSelector.get();
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.CommandSwerveFollowSpline;
import frc.robot.commands.CommandSwerveTeleopDrive;
import frc.robot.splines.PathFactory;
import frc.robot.splines.tasks.PerformAtTask;
import frc.robot.subsystems.SubsystemSwerveDrivetrain;
import frc.robot.test.TestManager;

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

  //
  // Subsystems
  //

  public SubsystemSwerveDrivetrain subsystemSwerveDrivetrain = new SubsystemSwerveDrivetrain();

  //
  // Commands
  //

  private CommandSwerveTeleopDrive commandSwerveTeleopDrive = new CommandSwerveTeleopDrive(subsystemSwerveDrivetrain,
      primaryController);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // we construct the DataManager instance here since it is the
    // absolute soonest we have access to a RobotContainer object
    new DataManager(this);

    // set sensible default commands
    setDefaultCommands();

    // H! Set all commands in auto selector
    setAutoCommands();

    // configure the controller bindings
    configureBindings();
  }

  /**
   * Used to set default commands for subsystems.
   */
  private void setDefaultCommands() {
    subsystemSwerveDrivetrain.setDefaultCommand(commandSwerveTeleopDrive);
  }

  private void setAutoCommands() {
    autoSelector
      .add(new InstantCommand(() -> {System.out.println("\n\n\nPRINT COMMAND RUN!\n\n\n");}), "Print Command")
      .add(new InstantCommand(() -> {System.out.println("\n\n\nOTHER COMMAND RUN!\n\n\n");}), "Other Command");
  }

  /**
   * Used to configure controller bindings.
   */
  private void configureBindings() {
    PIDController xController = new PIDController(1.2, 0, 0.1);
    PIDController yController = new PIDController(1.2, 0, 0.1);
    ProfiledPIDController thetaController = new ProfiledPIDController(0.7, 0, 0.0,
        new Constraints(3 * Math.PI, 6 * Math.PI));

    CommandSwerveFollowSpline followCommand = PathFactory.newFactory()
        .addPoint(1, 0)
        .addPoint(4, 0)
        .addPoint(4, -0.5)
        .addPoint(3, 0)
        .addTask(1, 0, new PerformAtTask(Rotation2d.fromDegrees(180), new InstantCommand()))
        .buildCommand(subsystemSwerveDrivetrain, xController, yController, thetaController);

    primaryController.a().onTrue(followCommand);
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

package frc.robot.commands.automatics;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants.ChassisKinematics;
import frc.robot.subsystems.SubsystemEndEffector;
import frc.robot.subsystems.SubsystemSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TaxiCommand extends Command {
  Timer timer = new Timer();
  SubsystemSwerveDrivetrain drivetrain;

  /** Creates a new CommandSwerveTaxi. */
  public TaxiCommand(SubsystemSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  public static Command L1AutoCommand(SubsystemSwerveDrivetrain drivetrain, SubsystemEndEffector endEffector) {
    TaxiCommand taxiCommand = new TaxiCommand(drivetrain);
    return taxiCommand.andThen(endEffector.continuousOuttakeCommand());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.8, 0, 0);
    SwerveModuleState[] moduleStates = ChassisKinematics.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    drivetrain.setModuleStates(moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, 0);
    SwerveModuleState[] moduleStates = ChassisKinematics.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    drivetrain.setModuleStates(moduleStates);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(3);
  }
}
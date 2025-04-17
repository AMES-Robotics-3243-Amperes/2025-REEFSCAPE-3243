// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorPositions;
import frc.robot.subsystems.SubsystemElevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorEjectCommand extends Command {

  SubsystemElevator elevator;
  /** Creates a new ElevatorJumpCommand. */
  public ElevatorEjectCommand(SubsystemElevator elevator) {
    this.elevator = elevator;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.rawNudge(1.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.rawNudge(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevator.getPosition() > ElevatorPositions.L3 && Math.abs(elevator.getVelocity()) < 0.05;
  }
}

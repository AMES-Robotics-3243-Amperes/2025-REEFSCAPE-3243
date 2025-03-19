// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SubsystemElevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorZeroCommand extends Command {
  private SubsystemElevator elevator;
  private final double speed = 0.30;
  private final double currentThreshold = 10.0;
  private int counter = 0;
  private final int timerLength = 50;
  /** Creates a new ElevatorZeroCommand. */
  public ElevatorZeroCommand(SubsystemElevator elevator) {
    this.elevator = elevator;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.rawNudge(-speed);
    if (elevator.getCurrent() > currentThreshold) {
      counter++;
    } else {
      counter = 0;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (counter > timerLength) {
      elevator.rezero();
      return true;
    }
    return false;
  }
}

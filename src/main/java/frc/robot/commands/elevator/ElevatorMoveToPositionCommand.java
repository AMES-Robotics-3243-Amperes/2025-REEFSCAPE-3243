// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SubsystemElevator;
import static frc.robot.Constants.Elevator.*;
import frc.robot.Constants.ElevatorPositions;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorMoveToPositionCommand extends Command {

  private SubsystemElevator elevator;
  private double target;
  private double deltaP;
  private double deltaV;

  public ElevatorMoveToPositionCommand(SubsystemElevator elevator, double target, double deltaP, double deltaV) {
    this.elevator = elevator;
    this.target = target;
    this.deltaP = deltaP;
    this.deltaV = deltaV;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
  }

  /** Creates a new ClimberMoveToPositionCommand. */
  public ElevatorMoveToPositionCommand(SubsystemElevator elevator, double target) {
    this(elevator, target, PositionChecking.deltaP, PositionChecking.deltaV);
  }

  public ElevatorMoveToPositionCommand(SubsystemElevator elevator, Position target) {
    this(elevator, target.position, PositionChecking.deltaP, PositionChecking.deltaV);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.setPosition(target);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return 
      Math.abs(elevator.getPosition() - target) < deltaP &&
      Math.abs(elevator.getVelocity()) < deltaV
    ;
  }

  public enum Position {
    Starting(ElevatorPositions.starting),
    Loading(ElevatorPositions.loading),
    L1(ElevatorPositions.L1),
    L2(ElevatorPositions.L2),
    L3(ElevatorPositions.L3),
    L4(ElevatorPositions.L4);

    public final double position;

    private Position(double position) {
      this.position = position;
    }
  }
}

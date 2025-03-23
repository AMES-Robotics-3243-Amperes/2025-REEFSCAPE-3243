// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.leds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DataManager;
import frc.robot.DataManager.ElevatorPositionData;
import frc.robot.subsystems.SubsystemLeds;
import frc.robot.subsystems.SubsystemLeds.Mode;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CommandLedsFromElevatorPosition extends Command {
  private ElevatorPositionData previousPosition = new ElevatorPositionData(-1, -1);
  private SubsystemLeds leds;
  private DataManager dataManager;
  /** Creates a new CommandLedsFromElevatorPosition. */
  public CommandLedsFromElevatorPosition(SubsystemLeds leds, DataManager dataManager) {
    this.leds = leds;
    this.dataManager = dataManager;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(leds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ElevatorPositionData position = dataManager.elevatorPosition.get();

    if (position.setpoint != previousPosition.setpoint) {
      switch (position.setpoint) {
        case Between:
          leds.setState(Mode.ElevatorMovingColor);
          break;
        case L1:
          leds.setState(Mode.L1Color);
          break;
        case L2:
          leds.setState(Mode.L2Color);
          break;
        case L3:
          leds.setState(Mode.L3Color);
          break;
        case L4:
          leds.setState(Mode.L4Color);
          break;
      }
    }

    previousPosition = position;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}

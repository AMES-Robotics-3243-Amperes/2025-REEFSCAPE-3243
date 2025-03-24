// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.automatics;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.DataManager;
import frc.robot.commands.elevator.ElevatorMoveToPositionCommand;
import frc.robot.commands.elevator.ElevatorNudgeCommand;
import frc.robot.subsystems.SubsystemElevator;
import frc.robot.subsystems.SubsystemEndEffector;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class L1DoubleHitScore extends SequentialCommandGroup {
  /** Creates a new L1DoubleHitScore. */
  public L1DoubleHitScore(SubsystemElevator elevator, SubsystemEndEffector endEffector, DataManager dataManager) {
    super
    (
      new ElevatorMoveToPositionCommand(elevator, ElevatorMoveToPositionCommand.Position.L1),
      new WaitCommand(0.15).deadlineFor(endEffector.continuousDriveCommand(0.15)),
      new WaitCommand(0.6).deadlineFor(endEffector.continuousDriveCommand(0.075)),
      new WaitUntilCommand(() -> dataManager.elevatorPosition.get().exactPos < 0.02).deadlineFor
      (
        new ElevatorNudgeCommand(elevator, -0.4),
        endEffector.continuousDriveCommand(0.22)
      ),
      new WaitCommand(0.5).deadlineFor(endEffector.continuousDriveCommand(0.2))
    );
  }
}

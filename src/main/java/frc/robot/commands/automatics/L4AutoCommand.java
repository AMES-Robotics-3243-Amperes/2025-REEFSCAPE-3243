package frc.robot.commands.automatics;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.elevator.ElevatorMoveToPositionCommand;
import frc.robot.commands.elevator.ElevatorMoveToPositionCommand.Position;
import frc.robot.subsystems.SubsystemElevator;
import frc.robot.subsystems.SubsystemEndEffector;
import frc.robot.subsystems.swerve.SubsystemSwerveDrivetrain;

public class L4AutoCommand extends SequentialCommandGroup {
  public L4AutoCommand(SubsystemSwerveDrivetrain drivetrain, SubsystemElevator elevator, SubsystemEndEffector endEffector) {
    super
    (
      new TaxiCommand(drivetrain, 3.0), 
      new ElevatorMoveToPositionCommand(elevator, Position.L4), 
      new WaitCommand(1.5).deadlineFor(endEffector.continuousOuttakeCommand())
    );
  }
}

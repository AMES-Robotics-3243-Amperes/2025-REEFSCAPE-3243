package frc.robot.commands.automatics;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SubsystemEndEffector;
import frc.robot.subsystems.SubsystemSwerveDrivetrain;

public class L1AutoCommand extends SequentialCommandGroup {
  public L1AutoCommand(SubsystemSwerveDrivetrain drivetrain, SubsystemEndEffector endEffector) {
    super(new TaxiCommand(drivetrain), endEffector.continuousOuttakeCommand());
  }
}

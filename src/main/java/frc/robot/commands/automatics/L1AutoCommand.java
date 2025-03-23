package frc.robot.commands.automatics;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SubsystemEndEffector;
import frc.robot.subsystems.swerve.SubsystemSwerveDrivetrain;

public class L1AutoCommand extends SequentialCommandGroup {
  public L1AutoCommand(SubsystemSwerveDrivetrain drivetrain, SubsystemEndEffector endEffector) {
    super(new TaxiCommand(drivetrain, 3.0), endEffector.continuousOuttakeCommand());
  }
}

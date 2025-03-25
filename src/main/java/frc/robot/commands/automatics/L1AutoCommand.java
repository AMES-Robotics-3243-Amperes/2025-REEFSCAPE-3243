package frc.robot.commands.automatics;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.DataManager;
import frc.robot.subsystems.SubsystemElevator;
import frc.robot.subsystems.SubsystemEndEffector;
import frc.robot.subsystems.swerve.SubsystemSwerveDrivetrain;

public class L1AutoCommand extends SequentialCommandGroup {
  public L1AutoCommand(SubsystemSwerveDrivetrain drivetrain, SubsystemElevator elevator, SubsystemEndEffector endEffector, DataManager dataManager) {
    super(new TaxiCommand(drivetrain, 3.0), new L1DoubleHitScore(elevator, endEffector, dataManager));
  }
}

package frc.robot.commands;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.swerve.SubsystemSwerveDrivetrain;

public class CommandSwerveGetOffset extends InstantCommand {
  SubsystemSwerveDrivetrain drivetrain;

  public CommandSwerveGetOffset(SubsystemSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    SwerveModulePosition[] positions = drivetrain.getAbsoluteModulePositions();
    System.out.println("Front Left Offset:" + positions[0].angle.getRadians());
    System.out.println("Front Right Offset:" + positions[1].angle.getRadians());
    System.out.println("Rear Left Offset:" + positions[2].angle.getRadians());
    System.out.println("Rear Right Offset:" + positions[3].angle.getRadians());
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}

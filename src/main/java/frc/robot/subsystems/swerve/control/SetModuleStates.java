package frc.robot.subsystems.swerve.control;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SetModuleStates extends SwerveControlRequest {
  SwerveModuleState[] states;

  private SetModuleStates(SwerveModuleState[] states) {
    this.states = states;

    limitSpeed = false;
    limitAccelaration = false;
  }

  public static SetModuleStates newRequest(SwerveModuleState[] states) {
    return new SetModuleStates(states);
  }

  @Override
  protected ControlOutput getRawOutput(Translation2d currentFieldOrientedVelocity, double secondsSinceLastControl) {
    return new ControlOutput(states);
  }
}

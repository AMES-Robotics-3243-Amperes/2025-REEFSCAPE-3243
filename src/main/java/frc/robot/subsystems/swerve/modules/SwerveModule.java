package frc.robot.subsystems.swerve.modules;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModule {
    public void setDesiredState(SwerveModuleState state);

    public void setDesiredRotation(Rotation2d roation);

    public SwerveModulePosition getPosition();

    public SwerveModulePosition getAbsolutePosition();

    public void update();

    public void driveVoltage(double voltage);
}

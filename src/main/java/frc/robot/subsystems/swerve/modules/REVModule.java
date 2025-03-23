// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve.modules;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog.MotorLog;
import frc.robot.Constants.SwerveConstants.ModuleConstants;
import frc.robot.Constants.SwerveConstants.ModuleConstants.PIDF;
import frc.robot.subsystems.swerve.SubsystemSwerveDrivetrain;

/**
 * Represents a single module of an {@link SubsystemSwerveDrivetrain}
 * 
 * @author :3
 */
public class REVModule implements SwerveModule {
  private final SparkMax m_drivingSparkMax;
  private final SparkMax m_turningSparkMax;

  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  private final SimpleMotorFeedforward m_drivingFeedforwardController;
  private final PIDController m_drivingPIDController;
  private double m_drivingVelocitySetpoint = 0;

  private final SparkClosedLoopController m_turningPIDController;

  private final Rotation2d m_wheelOffset;

  /**
   * Constructs a {@link ThriftyModule}
   * 
   * @param drivingCANId the id of the {@link CANSparkMax} for driving
   * @param turningCANId the id of the {@link CANSparkMax} for turning
   * @param wheelOffset  the offset of the encoder's 0 state
   * 
   * @author :3
   */
  public REVModule(int drivingCANId, int turningCANId, Rotation2d wheelOffset) {
    // :3 initialize spark maxes
    m_drivingSparkMax = new SparkMax(drivingCANId, MotorType.kBrushless);
    m_turningSparkMax = new SparkMax(turningCANId, MotorType.kBrushless);

    SparkMaxConfig drivingConfig = new SparkMaxConfig();
    SparkMaxConfig turningConfig = new SparkMaxConfig();

    // :3 setup pid and feedforward controllers
    m_drivingFeedforwardController = new SimpleMotorFeedforward(PIDF.kDrivingKs, PIDF.kDrivingKv, PIDF.kDrivingKv);
    m_drivingPIDController = new PIDController(PIDF.kDrivingP, PIDF.kDrivingI, PIDF.kDrivingD);

    // Use module constants to calculate conversion factors and feed forward gain.
    double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
        / ModuleConstants.kDrivingMotorReduction;
    double turningFactor = 2 * Math.PI;

    drivingConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(50);
    drivingConfig.encoder
        .positionConversionFactor(drivingFactor) // meters
        .velocityConversionFactor(drivingFactor / 60.0); // meters per second

    turningConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(20);
    turningConfig.absoluteEncoder
        // Invert the turning encoder, since the output shaft rotates in the opposite
        // direction of the steering motor in the MAXSwerve Module.
        .inverted(true)
        .positionConversionFactor(turningFactor) // radians
        .velocityConversionFactor(turningFactor / 60.0); // radians per second
    turningConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        // These are example gains you may need to them for your own robot!
        .pid(PIDF.kAzimuthP, PIDF.kAzimuthI, PIDF.kAzimuthD)
        .outputRange(-1, 1)
        // Enable PID wrap around for the turning motor. This will allow the PID
        // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
        // to 10 degrees will go through 0 rather than the other direction which is a
        // longer route.
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(0, turningFactor);

    m_drivingEncoder = m_drivingSparkMax.getEncoder();
    m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder();
    m_turningPIDController = m_turningSparkMax.getClosedLoopController();

    // finish up configuration
    m_drivingSparkMax.configure(drivingConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_turningSparkMax.configure(turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // :3 reset driving encoder
    resetEncoder();

    // :3 set wheel offset
    m_wheelOffset = wheelOffset;
  }

  /**
   * Sets the desired state of the module
   *
   * @param desiredState desired {@link SwerveModuleState}
   * 
   * @author :3
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // :3 apply wheel angular offset
    SwerveModuleState offsetState = new SwerveModuleState();
    offsetState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    offsetState.angle = desiredState.angle.plus(m_wheelOffset);
    offsetState.optimize(new Rotation2d(m_turningEncoder.getPosition()));

    // :3 command driving
    m_drivingVelocitySetpoint = offsetState.speedMetersPerSecond;
    m_turningPIDController.setReference(offsetState.angle.getRadians(), ControlType.kPosition);
  }

  /**
   * Sets the desired state of the module. Does not optimize states.
   *
   * @param desiredState desired {@link SwerveModuleState}
   * 
   * @author :3
   */
  public void setDesiredRotation(Rotation2d desiredRotation) {
    Rotation2d offsetRotation = desiredRotation.plus(m_wheelOffset);
    m_turningPIDController.setReference(offsetRotation.getRadians(), ControlType.kPosition);
  }

  /**
   * Updates the feedforward and PID controllers of the module.
   */
  public void update() {
    double drivingPIDOutput = m_drivingPIDController.calculate(m_drivingEncoder.getVelocity(),
        m_drivingVelocitySetpoint);
    double drivingFFOutput = m_drivingFeedforwardController.calculate(m_drivingVelocitySetpoint);
    m_drivingSparkMax.setVoltage(drivingPIDOutput + drivingFFOutput);
  }

  /**
   * Resets the module's driving encoder
   * 
   * @author :3
   */
  public void resetEncoder() {
    m_drivingEncoder.setPosition(0);
  }

  /**
   * @return the {@link SwerveModulePosition} of the module without the offset included
   * 
   * @author :3
   */
  public SwerveModulePosition getPosition() {
    double position = m_drivingEncoder.getPosition();
    Rotation2d rotation = Rotation2d.fromRadians(m_turningEncoder.getPosition() - m_wheelOffset.getRadians());

    return new SwerveModulePosition(position, rotation);
  }

  /**
   * @return the absolute {@link SwerveModulePosition} of the module (no offset)
   * 
   * @author :3
   */
  public SwerveModulePosition getAbsolutePosition() {
    double position = m_drivingEncoder.getPosition();
    Rotation2d rotation = Rotation2d.fromRadians(m_turningEncoder.getPosition());

    return new SwerveModulePosition(position, rotation);
  }

  /**
   * @return Gets the output motor current of an inputted motor
   * 
   * @author :>
   */
  public double getMotorOutputCurrent() {
    return m_drivingSparkMax.getOutputCurrent();
  }

  /**
   * Drives the module with the specified voltage for the drive motor,
   * and sets the target rotation for the pivot motor to 0.
   * 
   * @param voltage The voltage to set the drive motor to
   */
  public void driveVoltage(double voltage) {
    m_drivingSparkMax.setVoltage(voltage);
    setDesiredRotation(new Rotation2d(0));
  }

  /**
   * Logs all info for this motor. For use with SysID.
   * 
   * @param motorLog The {@link MotorLog} to log with
   */
  public void driveLog(MotorLog motorLog) {
    motorLog.current(Units.Amps.of(m_drivingSparkMax.getOutputCurrent()));
    motorLog.linearPosition(Units.Meters.of(m_drivingEncoder.getPosition()));
    motorLog.linearVelocity(Units.MetersPerSecond.of(m_drivingEncoder.getVelocity()));
    motorLog.voltage(Units.Volts.of(m_drivingSparkMax.getBusVoltage() * m_drivingSparkMax.getAppliedOutput()));
  }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve.modules;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SwerveConstants.ModuleConstants.PIDF;
import frc.robot.subsystems.swerve.SubsystemSwerveDrivetrain;

/**
 * Represents a single module of a {@link SubsystemSwerveDrivetrain}
 * 
 * @author :3
 */
public class ThriftyModule implements SwerveModule {
  private final TalonFX m_drivingTalonFX;
  private final SparkMax m_azimuthSparkMax;

  private final AnalogEncoder m_azimuthEncoder;

  private final PIDController m_azimuthPIDController;

  private double m_drivingVelocitySetpointMetersPerSecond = 0;
  private Rotation2d m_azimuthSetpoint = new Rotation2d();

  private final Rotation2d m_azimuthOffset;

  private boolean drivingWithRawVoltage = false;

  // TODO: constants!
  private final double wheelRadiusMeters = edu.wpi.first.math.util.Units.inchesToMeters(2);
  private final double drivingConversionFactor = 5.14;

  private final boolean doDebug;

  /**
   * Constructs a {@link ThriftyModule}
   * 
   * @param drivingCANId  the id of the {@link CANSparkMax} for driving
   * @param azimuthCANId  the id of the {@link CANSparkMax} for azimuth control
   * @param azimuthOffset the offset of the encoder's 0 state
   * 
   * @author :3
   */
  public ThriftyModule(int drivingCANId, int azimuthCANId, int encoderID, Rotation2d azimuthOffset) {
    doDebug = encoderID == 1;

    // :3 initialize spark maxes
    m_drivingTalonFX = new TalonFX(drivingCANId);
    m_azimuthSparkMax = new SparkMax(azimuthCANId, MotorType.kBrushless);

    m_azimuthEncoder = new AnalogEncoder(encoderID);
    m_azimuthPIDController = new PIDController(PIDF.kAzimuthP, PIDF.kAzimuthI, PIDF.kAzimuthD);
    m_azimuthPIDController.enableContinuousInput(-Math.PI, Math.PI);

    // configure motors
    TalonFXConfiguration drivingConfig = new TalonFXConfiguration();
    SparkMaxConfig azimuthConfig = new SparkMaxConfig();

    drivingConfig.CurrentLimits
        .withSupplyCurrentLimit(70) // TODO: constants!
        .withSupplyCurrentLimitEnable(true);
    drivingConfig.MotorOutput
        .withNeutralMode(NeutralModeValue.Coast);
    drivingConfig.Feedback
        .withSensorToMechanismRatio(drivingConversionFactor)
        .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);
    drivingConfig.MotionMagic
        .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(400)) // TODO: constants!
        .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(400));
    drivingConfig.Slot0
        .withKS(0)
        .withKV(0.6)
        .withKA(0)
        .withKP(0.2)
        .withKI(0)
        .withKD(0.05);

    azimuthConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(40);

    // finish up configuration
    m_azimuthSparkMax.clearFaults();
    m_azimuthSparkMax.configure(azimuthConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_drivingTalonFX.clearStickyFaults();
    m_drivingTalonFX.getConfigurator().apply(drivingConfig);

    // TODO: constants!
    m_drivingTalonFX.getVelocity().setUpdateFrequency(Frequency.ofRelativeUnits(200, Units.Hertz));
    m_drivingTalonFX.getPosition().setUpdateFrequency(Frequency.ofRelativeUnits(200, Units.Hertz));
    m_drivingTalonFX.getSupplyCurrent().setUpdateFrequency(Frequency.ofRelativeUnits(200, Units.Hertz));
    m_drivingTalonFX.getSupplyVoltage().setUpdateFrequency(Frequency.ofRelativeUnits(200, Units.Hertz));
    m_drivingTalonFX.optimizeBusUtilization();

    // set wheel offset
    m_azimuthOffset = azimuthOffset;
  }

  /**
   * Returns the azimuth encoder value as a {@link Rotation2d}.
   */
  private Rotation2d azimuthPosition() {
    return Rotation2d.fromRadians(MathUtil.angleModulus(m_azimuthEncoder.get() * 2 * Math.PI));
  }

  /**
   * Returns the driving position as a {@link Distance}
   */
  private Distance linearDrivingPosition() {
    double radians = m_drivingTalonFX.getPosition().getValue().in(Units.Radians);
    return Distance.ofRelativeUnits(radians * wheelRadiusMeters, Units.Meters);
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
    offsetState.angle = desiredState.angle.plus(m_azimuthOffset);
    offsetState.optimize(azimuthPosition());

    m_drivingVelocitySetpointMetersPerSecond = offsetState.speedMetersPerSecond;
    m_azimuthSetpoint = offsetState.angle;

    this.drivingWithRawVoltage = false;
  }

  /**
   * Sets the desired state of the module. Does not optimize states.
   *
   * @param desiredState desired {@link SwerveModuleState}
   * 
   * @author :3
   */
  public void setDesiredRotation(Rotation2d desiredRotation) {
    Rotation2d offsetRotation = desiredRotation.plus(m_azimuthOffset);
    m_azimuthSetpoint = offsetRotation;
  }

  /**
   * Updates the feedforward and PID controllers of the module.
   */
  public void update() {
    double desiredSpeedMetersPerSecond = m_drivingVelocitySetpointMetersPerSecond
        * Math.cos(m_azimuthSetpoint.getRadians() - azimuthPosition().getRadians());

    if (!drivingWithRawVoltage) {
      AngularVelocity desiredAngularVelocity = RadiansPerSecond
          .of(desiredSpeedMetersPerSecond / wheelRadiusMeters);
      MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(desiredAngularVelocity);
      m_drivingTalonFX.setControl(request.withSlot(0));
    }

    double azimuthOutput = m_azimuthPIDController.calculate(azimuthPosition().getRadians(),
        MathUtil.angleModulus(m_azimuthSetpoint.getRadians()));
    m_azimuthSparkMax.setVoltage(Units.Volts.of(azimuthOutput));

    if (doDebug) {
      SmartDashboard.putNumber("fr encoder", azimuthPosition().getRadians());
      SmartDashboard.putNumber("fr goal", m_azimuthSetpoint.getRadians());
      SmartDashboard.putNumber("fr output", azimuthOutput);
    }
  }

  /**
   * @return the {@link SwerveModulePosition} of the module
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(linearDrivingPosition().in(Units.Meters), azimuthPosition().minus(m_azimuthOffset));
  }

  /**
   * @return the {@link SwerveModulePosition} of the module without the offset
   */
  public SwerveModulePosition getAbsolutePosition() {
    return new SwerveModulePosition(linearDrivingPosition().in(Units.Meters), azimuthPosition());
  }

  /**
   * @return The supplied motor current of the driving motor
   */
  public double getMotorOutputCurrent() {
    return m_drivingTalonFX.getSupplyCurrent().getValue().in(Units.Amps);
  }

  /**
   * Drives the module with the specified voltage for the drive motor,
   * and sets the target rotation for the pivot motor to 0.
   * 
   * @param voltage The voltage to set the drive motor to
   */
  public void driveVoltage(double voltage) {
    this.drivingWithRawVoltage = true;
    m_drivingVelocitySetpointMetersPerSecond = 0;

    VoltageOut request = new VoltageOut(voltage);
    m_drivingTalonFX.setControl(request.withOverrideBrakeDurNeutral(true));
  }
}
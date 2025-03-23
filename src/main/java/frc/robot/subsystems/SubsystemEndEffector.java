// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class SubsystemEndEffector extends SubsystemBase {
  public static class EndEffectorConstants {
    public static final int sparkMaxId = 13;

    public static final int stallCurrentLimit = 80;
    public static final int freeCurrentLimit = 30;

    public static final double holdingCoralCurrentMin = 50;
    public static final double holdingCoralMinTime = 0.081;
    public static final int outputCurrentPeriod = 10;

    public static final double intakePower = 0.15;
    public static final double outtakePower = 0.27;

    public static final double outtakeTimeSeconds = 3;
  }

  private final SparkMax sparkMax = new SparkMax(EndEffectorConstants.sparkMaxId, MotorType.kBrushless);
  private final Timer coralTimer = new Timer();

  /** Creates a new EndEffector. */
  public SubsystemEndEffector() {
    SparkMaxConfig config = new SparkMaxConfig();
    
    config
      .smartCurrentLimit(EndEffectorConstants.stallCurrentLimit, EndEffectorConstants.freeCurrentLimit)
      .idleMode(IdleMode.kBrake)
      .inverted(true)
      .idleMode(IdleMode.kBrake)
    ;

    config.signals
      .outputCurrentPeriodMs(EndEffectorConstants.outputCurrentPeriod)
    ;
    
    sparkMax.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    if (sparkMax.getOutputCurrent() < EndEffectorConstants.holdingCoralCurrentMin) {
      coralTimer.restart();
    }
  }

  public void setPower(double speed) {
    sparkMax.set(speed);
  }

  public void stop() {
    sparkMax.set(0);
  }

  public boolean hasCoral() {
    return coralTimer.hasElapsed(EndEffectorConstants.holdingCoralMinTime);
  }

  public Command intakeCommand() {
    return this.runOnce(() -> this.setPower(EndEffectorConstants.intakePower))
        .andThen(new WaitUntilCommand(this::hasCoral))
        .finallyDo(this::stop);
  }

  public Command continuousOuttakeCommand() {
    return this.run(() -> this.setPower(EndEffectorConstants.outtakePower))
        .finallyDo(this::stop);
  }

  public Command outtakeCommand() {
    return this.run(() -> this.setPower(EndEffectorConstants.outtakePower))
        .andThen(new WaitCommand(EndEffectorConstants.outtakeTimeSeconds))
        .finallyDo(this::stop);
  }

  public Command continuousDriveCommand(double power) {
    return this.run(() -> this.setPower(power))
        .finallyDo(this::stop);
  }
}

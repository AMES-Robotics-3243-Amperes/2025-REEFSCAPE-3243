// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// Decide if we want to use switch, or ultrasonic sensor, etc.
import edu.wpi.first.wpilibj.Ultrasonic;
import frc.robot.Constants.DifferentialArm;
import frc.robot.Constants.DifferentialArm.*;
import frc.robot.DataManager.Setpoint;

public class SubsystemClaw extends SubsystemBase {
  // Some sort of sensor or limit switch to detect the PVC pipe
  public Ultrasonic rangeFinder;

  // Differential motors
  private SparkMax rightMotor = new SparkMax(MotorIDs.rightID, MotorType.kBrushless); // Previously forward
  private SparkMax leftMotor = new SparkMax(MotorIDs.leftID, MotorType.kBrushless); // Previously reverse
  
  private SparkMaxConfig rightConfig = new SparkMaxConfig();
  private SparkMaxConfig leftConfig = new SparkMaxConfig();

  private DifferentialMotorGroup motorGroup;
  private PIDController pivotController;

  private double targetPivotPosition = convertRadiansToRotations(Setpoint.Start.angle);
  private double intakePower = 0.0;

  private AbsoluteEncoder pivotEncoder;

  private static class DifferentialMotorGroup {
    private MotorController rightMotor;
    private MotorController leftMotor;

    private double pivotOutput;
    private double rollerOutput;

    // Matrix used to calculate the required inputs
    private static Matrix<N2, N2> inverseDifferentialMatrix = new Matrix<N2, N2>(N2.instance, N2.instance, new double[] {
       1.0, 1.0,
       1.0, -1.0
    });

    // Class to represent the differential motors
    public DifferentialMotorGroup(MotorController motorForward, MotorController motorReverse) {
      this.rightMotor = motorForward;
      this.leftMotor = motorReverse;
    }

    // Calculates the rate at which to spin the motors based on where they should be
    // Basically, we calculate the required inputs given outputs
    private void update() {
      Matrix<N2, N1> mechanismOutputs = new Matrix<N2, N1>(N2.instance, N1.instance, new double[] {pivotOutput, rollerOutput});
      Vector<N2> mechanismInputs = new Vector<N2>(inverseDifferentialMatrix.times(mechanismOutputs));
      
      // Scales the motor values to be between 0 and 1
      double maxMotorOutput = Math.max(mechanismInputs.get(0), mechanismInputs.get(1));
      if (maxMotorOutput > 1.0) {
        mechanismInputs.div(maxMotorOutput);
      }

      rightMotor.set(mechanismInputs.get(0));
      leftMotor.set(mechanismInputs.get(1));
    }

    public void setPivotOutput(double speed) {
      pivotOutput = clamp(-1.0, 1.0, speed);
    }

    public void setRollerOutput(double speed) {
      rollerOutput = clamp(-1.0, 1.0, speed);
    }
  }

  // Sets the target position for the absolute encoder
  public void setOutsidePosition(double angle) {
    targetPivotPosition = convertRadiansToRotations(angle);
  }

  public void setIntakePower(double power) {
    intakePower = power;
  }

  // Helper function for setOutsidePosition()
  private double convertRadiansToRotations(double angle) {
    return (angle / (2 * Math.PI)) + DifferentialArm.encoderOffset;
  }

  /** Creates a new SubsystemEndAffectorDifferential. */
  public SubsystemClaw(/* Ultrasonic rangeFinder */) {
    // Might need to invert this motor
    rightConfig.inverted(true);
    rightConfig.smartCurrentLimit(5);
    rightConfig.idleMode(IdleMode.kBrake);
    rightConfig.absoluteEncoder.inverted(true);
    
    leftConfig.smartCurrentLimit(5);
    leftConfig.idleMode(IdleMode.kBrake);

    rightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    pivotEncoder = rightMotor.getAbsoluteEncoder();
    
    motorGroup = new DifferentialMotorGroup(rightMotor, leftMotor);
    pivotController = new PIDController(DifferentialArm.PID.P, DifferentialArm.PID.I, DifferentialArm.PID.D);

    Shuffleboard.getTab("Tuning").add(pivotController).withWidget(BuiltInWidgets.kPIDController);

    // Decide if we want to use switch, or ultrasonic sensor, etc.
    // this.rangeFinder = rangeFinder;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double pivotControllerCalculate = pivotController.calculate(pivotEncoder.getPosition(), targetPivotPosition);
    motorGroup.setPivotOutput(pivotControllerCalculate);
    motorGroup.setRollerOutput(intakePower);
    motorGroup.update();
    SmartDashboard.putNumber("Arm Absolute Encoder Rotations", pivotEncoder.getPosition());
    SmartDashboard.putNumber("Arm PID pivot controller output", pivotControllerCalculate);
    SmartDashboard.putNumber("Target position", targetPivotPosition);
  }

  private static double clamp(double min, double max, double x) {
    return Math.max(min, Math.min(max, x));
  }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DifferentialArm;
import frc.robot.DataManager.Setpoint;

public class SubsystemClaw extends SubsystemBase {
  // Some sort of sensor or limit switch to detect the PVC pipe
  // public DigitalInput limitSwitch;

  private ShuffleboardTab tab = Shuffleboard.getTab("Deepwater (Claw)");

  // Differential motors
  private SparkMax rightMotor = new SparkMax(DifferentialArm.MotorIDs.rightID, MotorType.kBrushless);
  private SparkMax leftMotor = new SparkMax(DifferentialArm.MotorIDs.leftID, MotorType.kBrushless);

  private SparkMaxConfig rightConfig = new SparkMaxConfig();
  private SparkMaxConfig leftConfig = new SparkMaxConfig();

  // Motor group and PID controller to control movement via Matrix multiplication and PID control
  private DifferentialMotorGroup motorGroup;
  private PIDController pivotController;

  // Starting values for the arm
  public double targetPivotPosition = Setpoint.Start.angle;
  private double intakePower = 0;

  // Exponentially smoothing linear filter to smooth the current difference
  LinearFilter filter = LinearFilter.singlePoleIIR(DifferentialArm.filterTimeConstant, 0.02);
  public double smoothedCurrentDifference;

  // Gravity compensation (secretly feedforward)
  private double gravityCompensation = Constants.DifferentialArm.defaultGravityCompensation;

  private AbsoluteEncoder pivotEncoder;
  // private ArmFeedforward feedforward;

  // Nested class to perform updates to the linear system through matrix multiplication
  private static class DifferentialMotorGroup {
    private MotorController rightMotor;
    private MotorController leftMotor;

    private double pivotOutput;
    private double rollerOutput;

    // Matrix used to calculate the required inputs
    private static Matrix<N2, N2> inverseDifferentialMatrix = new Matrix<N2, N2>(N2.instance, N2.instance,
        new double[] {
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
      Matrix<N2, N1> mechanismOutputs = new Matrix<N2, N1>(N2.instance, N1.instance,
          new double[] { pivotOutput, rollerOutput });
      Vector<N2> mechanismInputs = new Vector<N2>(inverseDifferentialMatrix.times(mechanismOutputs));

      // Scales the motor values to be between 0 and 1
      double maxMotorOutput = Math.max(mechanismInputs.get(0), mechanismInputs.get(1));
      if (maxMotorOutput > 1.0) {
        mechanismInputs.div(maxMotorOutput);
      }

      rightMotor.set(mechanismInputs.get(0));
      leftMotor.set(mechanismInputs.get(1));
    }

    public void setPivotOutput(double angle) {
      pivotOutput = MathUtil.clamp(angle, -0.4, 0.4);
    }

    public void setRollerOutput(double speed) {
      rollerOutput = MathUtil.clamp(speed, -1.0, 1.0);
    }
  }

  // Sets the target position for the absolute encoder
  public void setOutsidePosition(double angle) {
    targetPivotPosition = angle;
  }

  // You'll never guess...
  public void setIntakePower(double power) {
    intakePower = power;
  }

  public double getPosition() {
    return targetPivotPosition;
  }

  public void resetFilter() {
    filter.reset();
  }

  // Helper function for setOutsidePosition()
  // private double convertRadiansToRotations(double angle) {
  //   return (angle / (2 * Math.PI)) + DifferentialArm.encoderOffset;
  // }

  /** Creates a new SubsystemEndAffectorDifferential. */
  public SubsystemClaw(/* Ultrasonic rangeFinder */) {
    rightConfig.inverted(true);
    rightConfig.smartCurrentLimit(25);
    rightConfig.idleMode(IdleMode.kBrake);
    rightConfig.absoluteEncoder.inverted(true);

    leftConfig.smartCurrentLimit(25);
    leftConfig.idleMode(IdleMode.kBrake);

    rightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    pivotEncoder = rightMotor.getAbsoluteEncoder();

    motorGroup = new DifferentialMotorGroup(rightMotor, leftMotor);
    pivotController = new PIDController(DifferentialArm.PID.P, DifferentialArm.PID.I, DifferentialArm.PID.D);
    pivotController.reset();

    // limitSwitch = new DigitalInput(0);

    Shuffleboard.getTab("Tuning").add(pivotController).withWidget(BuiltInWidgets.kPIDController);

    tab.addDouble("Arm Absolute Encoder Rotations", () -> pivotEncoder.getPosition());
    tab.addDouble("Arm PID pivot controller output", () -> pivotControllerCalculate);
    tab.addDouble("Arm gravity compensation", () -> staticTerm);
    tab.addDouble("Arm applied output", () -> staticTerm + pivotControllerCalculate);
    tab.addDouble("Target position", () -> targetPivotPosition);
    tab.addDouble("Intake power", () -> intakePower);

    tab.addDouble("Left motor value", () -> leftMotor.get());
    tab.addDouble("Right motor value", () -> rightMotor.get());

    tab.addDouble("Left motor temp", () -> leftMotor.getMotorTemperature());
    tab.addDouble("Right motor temp", () -> rightMotor.getMotorTemperature());
    
    tab.addDouble("Right Motor current", () -> rightMotor.getOutputCurrent());
    tab.addDouble("Left Motor current", () -> leftMotor.getOutputCurrent());
    tab.addDouble("Smoothed motor difference", () -> smoothedCurrentDifference);
  }

  // Only not local so that it can be seen by supplied in constructor
  private double pivotControllerCalculate = 0.0; 
  private double staticTerm = 0.0;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    pivotControllerCalculate = pivotController.calculate(pivotEncoder.getPosition(), targetPivotPosition);

    smoothedCurrentDifference = filter.calculate(rightMotor.getOutputCurrent() - leftMotor.getOutputCurrent());

    motorGroup.setPivotOutput(pivotControllerCalculate);
    motorGroup.setRollerOutput(intakePower);
    motorGroup.update();
  }
}
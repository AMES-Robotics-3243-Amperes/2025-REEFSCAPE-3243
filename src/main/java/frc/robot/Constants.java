// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import org.photonvision.PhotonCamera;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.robot.splines.interpolation.LinearInterpolator;
import frc.robot.splines.interpolation.SplineInterpolator;
import frc.robot.utility.PhotonCameraGroup;
import frc.robot.utility.PhotonCameraGroup.PhotonCameraSetup;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class JoyUtilConstants {
    public static final double exponent1 = 3;
    public static final double exponent2 = 1;
    public static final double coeff1 = 0.4;
    public static final double coeff2 = 0.6;
  }

  public static final class SwerveConstants {
    public static final class ControlConstants {
      public static final double movingSpeed = 0.5;
      public static final double maxSpeed = 5;
      public static final double maxSpeedAtMaxElevatorExtension = 0.2;

      public static final double accelerationLimitSafetyFactor = 1.5;
      public static final double baseCenterOfMass = Units.inchesToMeters(12 - 1);
      public static final double baseAcceleration = Units.inchesToMeters(17) * 9.8 / baseCenterOfMass; // hale, if you're reading this, this is alpha / beta // Got it, thanks!
      public static final double percentOfWeightInElevator = Units.inchesToMeters(21 - 12);

      public static final double rotationSpeed = 1.2 * Math.PI;

      public static final double leftTriggerMultiplier = 2.5;
      public static final double rightTriggerMultiplier = 0.3;
    }

    public static final class ChassisKinematics {
      // :3 distance between centers of right and left wheels on robot
      public static final double kRobotWidth = Units.inchesToMeters(26);
      // :3 distance between front and back wheels on robot
      public static final double kRobotLength = Units.inchesToMeters(26);

      public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
          new Translation2d(kRobotLength / 2, kRobotWidth / 2),
          new Translation2d(kRobotLength / 2, -kRobotWidth / 2),
          new Translation2d(-kRobotLength / 2, kRobotWidth / 2),
          new Translation2d(-kRobotLength / 2, -kRobotWidth / 2));
    }

    public static final class DriveTrainConstants {
      public static final class IDs {
        public static final int kFrontLeftDrivingCanId = 1;
        public static final int kRearLeftDrivingCanId = 3;
        public static final int kFrontRightDrivingCanId = 2;
        public static final int kRearRightDrivingCanId = 4;

        public static final int kFrontLeftTurningCanId = 1;
        public static final int kRearLeftTurningCanId = 3;
        public static final int kFrontRightTurningCanId = 2;
        public static final int kRearRightTurningCanId = 4;
      }

      public static final class ModuleOffsets {
        public static final Rotation2d kFrontLeftOffset = Rotation2d.fromRadians(-2.224);
        public static final Rotation2d kFrontRightOffset = Rotation2d.fromRadians(-0.178);
        public static final Rotation2d kBackLeftOffset = Rotation2d.fromRadians(6.195);
        public static final Rotation2d kBackRightOffset = Rotation2d.fromRadians(-2.763);
      }
    }

    public static final class ModuleConstants {
      public static final class PIDF {
        public static final double kDrivingP = 0.0;
        public static final double kDrivingI = 0;
        public static final double kDrivingD = 0;

        public static final double kDrivingKs = 0.01;
        public static final double kDrivingKv = 0.11324;
        public static final double kDrivingKa = 0.034615;

        public static final double kAzimuthP = 4.2;
        public static final double kAzimuthI = 0;
        public static final double kAzimuthD = 0.0;
        public static final double kTurningFF = 0;
        public static final double kTurningMinOutput = -1;
        public static final double kTurningMaxOutput = 1;
      }

      public static final double kMaxObtainableModuleSpeed = 100;

      // The MAXSwerve module can be configured with one of three pinion gears: 12T,
      // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
      // more teeth will result in a robot that drives faster).
      public static final int kDrivingMotorPinionTeeth = 14;

      // Calculations required for driving motor conversion factors and feed forward
      public static final double kDrivingMotorFreeSpeedRps = 5676.0 / 60.0;
      public static final double kWheelDiameterMeters = 0.0762;
      public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
      // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
      // teeth on the bevel pinion
      public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
      public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
          / kDrivingMotorReduction;
    }
  }

  public static class Elevator {
    public static final double manualThreshold = 0.1;

    public static class Motors {
      public static final int leaderCanId = 10; // Leader is the right motor, update this if that changes
      public static final int followerCanId = 11; // Follower is the left motor, update this if that changes

      public static final double positionConversionRatio = Math.pow(18.0 / 50.0, 2) * // Gear ratio
          22 * // Number of sprocket teeth
          Units.inchesToMeters(1.0 / 4.0) * // Distance between chain links
          3.0 // Elevator stages
      ;

      public static final double velocityConversionRatio = (1.0 / 60.0) * // Convert RPM to RPS
          positionConversionRatio // The normal stuff
      ;

      public static final double P = 3;
      public static final double I = 0.0;
      public static final double D = 1;
      public static final double FF = 3;
    }

    public static class PositionChecking {
      public static final double deltaP = 0.05;
      public static final double deltaV = 0.05;
    }

    public static class Control {
      public static final double upNudgeVelocity = 0.25;
      public static final double downNudgeVelocity = -0.25;
    }

    public static class SpeedSettings {
      public static final double highSpeed = 1.0;
      public static final double midSpeed = 0.5;
      public static final double lowSpeed = 0.2;
    }
  }

  public static class Climber {
    public static final int climberCanId = 0;
  }

  /**
   * Positions are measured by the pivot position height above minimum. All
   * heights are in meters.
   */
  public static class ElevatorPositions {
    /**
     * The vertical distance from the ground to the minimum height on the elevator.
     */
    public static final double angledOffset23 = 6.85; // 35 degrees
    public static final double angledOffset4 = 10.875; // 65 degrees

    public static final double min = 0.0;
    public static final double starting = min;
    public static final double store = 0.03;
    public static final double loading = 0.6;// Units.inchesToMeters(37 - angledOffset23);
    public static final double L1 = 0.225;// Units.inchesToMeters(18 + angledOffset23);
    public static final double L2 = 0.525;// Units.inchesToMeters(31.2 + angledOffset23);
    public static final double L3 = 0.93;// Units.inchesToMeters(47.025 + angledOffset23);
    public static final double L4 = 1.57;// Units.inchesToMeters(72 + angledOffset4);
    public static final double A1 = 0.786; // First Algae
    public static final double A2 = 1.142; // Second Algae
    public static final double max = 1.60;// Units.inchesToMeters(85);
  }

  public static final class FieldConstants {

    /*
     * <3 Like we did last year, I'll define forwards to be facing towards the red
     * alliance
     * (0,0) is the bottom left corner of the field if you're looking from a
     * top-down perspective with blue alliance on your left-hand side.
     * Positive x is towards red alliance. Positive Y will be left when you are
     * facing the red alliance.
     * Measurements are in meters.
     */

    // <3 Positions of Apriltags
    public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout
        .loadField(AprilTagFields.k2025ReefscapeWelded);

    public static final Pose3d blueCoralLoadingTop = fieldLayout.getTagPose(13).get();
    public static final Pose3d blueCoralLoadingBottom = fieldLayout.getTagPose(12).get();

    public static final Pose3d redCoralLoadingTop = fieldLayout.getTagPose(2).get();
    public static final Pose3d redCoralLoadingBottom = fieldLayout.getTagPose(1).get();

    public static final Pose3d blueAlgaeProcessor = fieldLayout.getTagPose(3).get();
    public static final Pose3d redAlgaeProcessor = fieldLayout.getTagPose(16).get();

    public static final Pose3d blueBargeLeft = fieldLayout.getTagPose(14).get();
    public static final Pose3d blueBargeRight = fieldLayout.getTagPose(4).get();

    public static final Pose3d redBargeLeft = fieldLayout.getTagPose(15).get();
    public static final Pose3d redBargeRight = fieldLayout.getTagPose(5).get();

    public static final Pose3d blueReef1 = fieldLayout.getTagPose(19).get();
    public static final Pose3d blueReef2 = fieldLayout.getTagPose(20).get();
    public static final Pose3d blueReef3 = fieldLayout.getTagPose(21).get();
    public static final Pose3d blueReef4 = fieldLayout.getTagPose(22).get();
    public static final Pose3d blueReef5 = fieldLayout.getTagPose(17).get();
    public static final Pose3d blueReef6 = fieldLayout.getTagPose(18).get();

    public static final Pose3d redReef1 = fieldLayout.getTagPose(8).get();
    public static final Pose3d redReef2 = fieldLayout.getTagPose(9).get();
    public static final Pose3d redReef3 = fieldLayout.getTagPose(10).get();
    public static final Pose3d redReef4 = fieldLayout.getTagPose(11).get();
    public static final Pose3d redReef5 = fieldLayout.getTagPose(6).get();
    public static final Pose3d redReef6 = fieldLayout.getTagPose(7).get();

    public static final List<AprilTag> tagList = fieldLayout.getTags();
    public static final double distanceFromTag = Units.inchesToMeters(26 / 2); // Length of Robot Divided by 2

    // <3 Measurements taken from CAD--center axis dist. from apriltag to estimated
    // robot center position
    public static final double reefScoreOffset = 0.181;
    public static final double intakeLoadingOffset = 0.457;
  }

  public static final class PhotonvisionConstants {

    public static final PhotonCameraGroup cameraGroup = new PhotonCameraGroup
    (
      new PhotonCameraSetup
      (
        new PhotonCamera("FrontLeftCamera"),
        new Transform3d
        (
          new Translation3d(0.072702168663761, 0.32809278061041536, 0.20121528823603776),
          new Rotation3d(0.06280283225338665, -0.03371785334821224, -0.7028873363435759)
        )
      )
    );

    public static final double photonUnitAmbiguityCutoff = 0.1;

    public static final double poseEstimatorAmbiguityScaleFactor = 2;
    public static final double photonUnitVelocityCutoff = 1.0;
    public static final double photonUnitMinDistance = 0.4;
  }

  public static final class SplineConstants {
    public static final class NumericalConstants {
      public static final int compositeGaussianQuadratureIntervals = 3;
      public static final int newtonRaphsonIterations = 10;
    }

    public static final class TaskConstants {
      public static final Rotation2d defaultRotationTolerance = Rotation2d.fromDegrees(2.5);
      public static final double defaultPositionTolerance = Units.inchesToMeters(0.3);
      public static final double defaultPositionBuffer = 0.3;
    }

    public static final class FollowConstants {
      public static final SplineInterpolator defaultInterpolator = new LinearInterpolator(); // Could change to cubic
      public static final double maxSpeed = 0.8;
      public static final double maxCentrifugalAcceleration = 2;
      public static final double maxAccelAfterTask = 1.5;
      public static final boolean interpolateFromStart = true;

      public static final double staticThetaVelocity = 0.2;

      /**
       * Returns a sensible default x/y PID controller for spline following
       */
      public static final PIDController xyController() {
        return new PIDController(1.2, 0, 0.1);
      }

      /**
       * Returns a sensible default theta PID controller for spline following
       */
      public static final ProfiledPIDController thetaController() {
        ProfiledPIDController thetaController = new ProfiledPIDController(1.6, 0, 0,
            new Constraints(50 * Math.PI, 50 * Math.PI));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        thetaController.setIZone(3 * Math.PI / 16);
        return thetaController;
      }

      /**
       * As the robot drifts from the spline, the speed at
       * which the setpoint travels across the spline decreases.
       * This function determines the speed multiplier as a function
       * of robot offset from the spline.
       */
      public static final double splineOffsetVelocityDampen(double offset) {
        return 1 / (1 + 1.5 * offset * offset);
      }

      /**
       * To avoid harsh acceleration, slow the robot's movement as it starts following
       * the path. This function gives a hard velocity cap as a function of length
       * traversed.
       */
      public static final double splineStartVelocityDampen(double length) {
        return 5 * length + 0.2;
      }

      /**
       * To avoid harsh stops, slow the robot's movement as it
       * finishes the path. This function gives a hard velocity
       * cap as a function of remaining length.
       */
      public static final double splineCompleteVelocityDampen(double remainingLength) {
        return 5 * remainingLength + 0.2;
      }

      /**
       * The robot needs to slow down to ensure it executes tasks in the correct
       * position. This command dictates the max velocity of the robot as a function
       * of remaining valid length to execute a task.
       */
      public static final double splineTaskVelocityDampen(double remainingLength) {
        return 0.8 * remainingLength + 0.1;
      }
    }
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.robot.DataManager.Setpoint;
import frc.robot.splines.interpolation.CubicInterpolator;
import frc.robot.splines.interpolation.LinearInterpolator;
import frc.robot.splines.interpolation.SplineInterpolator;

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
    public static final double kDeadzone = 0.05;
    public static final double kRateLimitLeft = 65;
    public static final double kRateLimitRight = 65;
    public static final double exponent1 = 3;
    public static final double exponent2 = 1;
    public static final double coeff1 = 0.2;
    public static final double coeff2 = 0.8;

    public static final double leftTriggerSpeedMultiplier = 2.5;
    public static final double rightTriggerSpeedMultiplier = 0.3;
  }

  public static final class SwerveConstants {
    public static final class ControlConstants {
      public static final double movingSpeed = 1.2;
      public static final double rotationSpeed = 1.2 * Math.PI;
    }

    public static final class ChassisKinematics {
      // :3 distance between centers of right and left wheels on robot
      public static final double kRobotWidth = Units.inchesToMeters(15);
      // :3 distance between front and back wheels on robot
      public static final double kRobotLength = Units.inchesToMeters(15);

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
      public static final double upNudgeVelocity = 1.0;
      public static final double downNudgeVelocity = -1.0;
    }

    public static class SpeedSettings {
      public static final double highSpeed = 1.0;
      public static final double midSpeed = 0.5;
      public static final double lowSpeed = 0.2;
    }
  }

  public static class Climber{
    public static final int climberCanId = 0;
  }

  /**
   * Positions are measured by the pivot position height above minimum. All
   * heights are in meters.
   */
  public static class Positions {
    /**
     * The vertical distance from the ground to the minimum height on the elevator.
     */
    public static final double angledOffset23 = 6.85; // 35 degrees
    public static final double angledOffset4 = 10.875; // 65 degrees

    public static final double min = 0.0;
    public static final double starting = min;
    public static final double store = 0.05;
    public static final double loading = 0.6;//Units.inchesToMeters(37 - angledOffset23);
    public static final double L1 = 0.4;//Units.inchesToMeters(18 + angledOffset23);
    public static final double L2 = 0.64;//Units.inchesToMeters(31.2 + angledOffset23);
    public static final double L3 = 1.05;//Units.inchesToMeters(47.025 + angledOffset23);
    public static final double L4 = 1.74;//Units.inchesToMeters(72 + angledOffset4);
    public static final double A1 = 0.786; // First Algae
    public static final double A2 = 1.142; // Second Algae
    public static final double max = 1.74;//Units.inchesToMeters(85);
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

    /*
     * <3 The positions of the algae on the ground of the blue alliance side of the
     * field
     * public static Pose2d blueGroundAlgae1 = new Pose2d( -7.555, 1.829, new
     * Rotation2d(0));
     * public static Pose2d blueGroundAlgae2 = new Pose2d( -7.555, 0.000, new
     * Rotation2d(0));
     * public static Pose2d blueGroundAlgae3 = new Pose2d( -7.555, -1.829, new
     * Rotation2d(0));
     * 
     * // <3 The positions of the algae on the ground of the red alliance side of
     * the field
     * public static Pose2d redGroundAlgae1 = new Pose2d(7.555, 1.829, new
     * Rotation2d(0));
     * public static Pose2d redGroundAlgae2 = new Pose2d(7.555, 0.000, new
     * Rotation2d(0));
     * public static Pose2d redGroundAlgae3 = new Pose2d(7.555, -1.829, new
     * Rotation2d(0));
     * 
     * // <3 The positions of the blue alliance's reef posts, measured from the
     * bottom left corner of the field to the center of the pole
     * public static Pose2d blueReefPole1 = new Pose2d(6.587, 4.946, new
     * Rotation2d(0));
     * public static Pose2d blueReefPole2 = new Pose2d(6.872, 5.109, new
     * Rotation2d(0));
     * public static Pose2d blueReefPole3 = new Pose2d(7.114, 5.110, new
     * Rotation2d(0));
     * public static Pose2d blueReefPole4 = new Pose2d(7.398, 4.945, new
     * Rotation2d(0));
     * public static Pose2d blueReefPole5 = new Pose2d(7.520, 4.736, new
     * Rotation2d(0));
     * public static Pose2d blueReefPole6 = new Pose2d(7.518, 4.408, new
     * Rotation2d(0));
     * public static Pose2d blueReefPole7 = new Pose2d(7.398, 4.198, new
     * Rotation2d(0));
     * public static Pose2d blueReefPole8 = new Pose2d(7.113, 4.035, new
     * Rotation2d(0));
     * public static Pose2d blueReefPole9 = new Pose2d(6.872, 4.034, new
     * Rotation2d(0));
     * public static Pose2d blueReefPole10 = new Pose2d(6.588, 4.199, new
     * Rotation2d(0));
     * public static Pose2d blueReefPole11 = new Pose2d(6.466, 4.408, new
     * Rotation2d(0));
     * public static Pose2d blueReefPole12 = new Pose2d(6.467, 4.736, new
     * Rotation2d(0));
     * 
     * // <3 The positions of the red alliance's reef posts, measured from the
     * bottom left corner of the field to the center of the pole
     * public static Pose2d redReefPole1 = new Pose2d(15.157, 4.946, new
     * Rotation2d(0));
     * public static Pose2d redReefPole2 = new Pose2d(15.442, 5.109, new
     * Rotation2d(0));
     * public static Pose2d redReefPole3 = new Pose2d(15.683, 5.110, new
     * Rotation2d(0));
     * public static Pose2d redReefPole4 = new Pose2d(15.967, 4.945, new
     * Rotation2d(0));
     * public static Pose2d redReefPole5 = new Pose2d(16.089, 4.736, new
     * Rotation2d(0));
     * public static Pose2d redReefPole6 = new Pose2d(16.088, 4.408, new
     * Rotation2d(0));
     * public static Pose2d redReefPole7 = new Pose2d(15.968, 4.198, new
     * Rotation2d(0));
     * public static Pose2d redReefPole8 = new Pose2d(15.683, 4.035, new
     * Rotation2d(0));
     * public static Pose2d redReefPole9 = new Pose2d(15.441, 4.034, new
     * Rotation2d(0));
     * public static Pose2d redReefPole10 = new Pose2d(15.157, 4.199, new
     * Rotation2d(0));
     * public static Pose2d redReefPole11 = new Pose2d(15.036, 4.408, new
     * Rotation2d(0));
     * public static Pose2d redReefPole12 = new Pose2d(15.037, 4.736, new
     * Rotation2d(0));
     * 
     * // <3 The positions of the blue alliance's coral loading stations. There are
     * positions for two of the slots on each station.
     * // <3 The slot positions for each loading station are labelled with A being
     * the third slot and B being the eighth slot if you were standing on the field,
     * looking at the coral station.
     * public static Pose2d blueCoralLoading1A = new Pose2d(-3.756, -0.890, new
     * Rotation2d(0));
     * public static Pose2d blueCoralLoading1B = new Pose2d(-2.934, -1.487, new
     * Rotation2d(0));
     * 
     * public static Pose2d blueCoralLoading2A = new Pose2d(-2.934, -7.657, new
     * Rotation2d(0));
     * public static Pose2d blueCoralLoading2B = new Pose2d(-3.756, -8.254, new
     * Rotation2d(0));
     * 
     * // <3 The positions of the red alliance's coral loading stations. There are
     * positions for two of the slots on each station.
     * // <3 The slot positions for each loading station are labelled with A being
     * the third slot and B being the eighth slot if you were standing on the field,
     * looking at the coral station.
     * public static Pose2d redCoralLoading1A = new Pose2d(-19.621, -1.487, new
     * Rotation2d(0));
     * public static Pose2d redCoralLoading1B = new Pose2d(-18.799, -0.890, new
     * Rotation2d(0));
     * 
     * public static Pose2d redCoralLoading2A = new Pose2d(-18.799, -8.254, new
     * Rotation2d(0));
     * public static Pose2d redCoralLoading2B = new Pose2d(-19.621, -7.657, new
     * Rotation2d(0));
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

    public static final class AutonomousPaths {
      // Testing autonomous
      public static final ArrayList<Pair<Setpoint, Boolean>> testingSetpoints = new ArrayList<Pair<Setpoint, Boolean>>(
        Arrays.asList(Pair.of(Setpoint.L4Left, true))
      );

      public static final ArrayList<Pose2d> testingPositions = new ArrayList<Pose2d>(
        Arrays.asList(FieldConstants.blueReef4.toPose2d()));
      
      // L4 and Sideways
      public static final ArrayList<Pair<Setpoint, Boolean>> l4ThenSidewaysSetpoints = new ArrayList<Pair<Setpoint, Boolean>>(
        Arrays.asList(Pair.of(Setpoint.L4Left, true), Pair.of(Setpoint.Start, false))
      );

      public static final ArrayList<Pose2d> l4ThenSidewaysPositions = new ArrayList<Pose2d>(
        Arrays.asList(FieldConstants.blueReef3.toPose2d(),
        FieldConstants.blueReef3.toPose2d().transformBy(new Transform2d(0, Units.inchesToMeters(50), DataManager.instance().robotPosition.get().getRotation())))
      );

      // L4 and Intake
      public static final ArrayList<Pair<Setpoint, Boolean>> l4ThenIntakeSetpoints = new ArrayList<Pair<Setpoint, Boolean>>(
        Arrays.asList(Pair.of(Setpoint.L4Left, true), Pair.of(Setpoint.Start, false), Pair.of(Setpoint.IntakeLeft, false))
      );

      public static final ArrayList<Pose2d> l4ThenIntakePositions = new ArrayList<Pose2d>(
        Arrays.asList(FieldConstants.redReef3.toPose2d(),
        FieldConstants.blueReef3.toPose2d().transformBy(new Transform2d(0, Units.inchesToMeters(50), DataManager.instance().robotPosition.get().getRotation())),
        FieldConstants.blueCoralLoadingTop.toPose2d())
      );

      // Scoring/Intake Setpoints
      public static final ArrayList<Pair<Setpoint, Boolean>> intakeScoreBackAndForthSetpoints = new ArrayList<Pair<Setpoint, Boolean>>(
        Arrays.asList(Pair.of(Setpoint.L4Left, true), Pair.of(Setpoint.IntakeLeft, false),
        Pair.of(Setpoint.L4Right, true), Pair.of(Setpoint.IntakeRight, true))
      );

      // Top of blue reef to intake auto routine
      public static final ArrayList<Pose2d> blueTopToIntakePositions = new ArrayList<Pose2d>(
          Arrays.asList(FieldConstants.blueReef1.toPose2d(),
              FieldConstants.blueCoralLoadingTop.toPose2d(), FieldConstants.blueReef1.toPose2d(),
              FieldConstants.blueCoralLoadingTop.toPose2d()));

      // Middle of blue reef to top intake auto routine
      public static final ArrayList<Pose2d> blueMiddleToTopIntakePositions = new ArrayList<Pose2d>(
        Arrays.asList(FieldConstants.blueReef3.toPose2d(),
            FieldConstants.blueCoralLoadingTop.toPose2d(), FieldConstants.blueReef1.toPose2d(),
            FieldConstants.blueCoralLoadingTop.toPose2d()));

      // Middle of blue reef to bottom intake auto routine
      public static final ArrayList<Pose2d> blueMiddleToBottomIntakePositions = new ArrayList<Pose2d>(
        Arrays.asList(FieldConstants.blueReef3.toPose2d(),
            FieldConstants.blueCoralLoadingBottom.toPose2d(), FieldConstants.blueReef5.toPose2d(),
            FieldConstants.blueCoralLoadingBottom.toPose2d()));

      // Bottom of blue reef to intake auto routine
      public static final ArrayList<Pose2d> blueBottomToIntakePositions = new ArrayList<Pose2d>(
        Arrays.asList(FieldConstants.blueReef5.toPose2d(),
            FieldConstants.blueCoralLoadingBottom.toPose2d(), FieldConstants.blueReef5.toPose2d(),
            FieldConstants.blueCoralLoadingBottom.toPose2d()));

      // Top of red reef to intake auto routine
      public static final ArrayList<Pose2d> redTopToIntakePositions = new ArrayList<Pose2d>(
          Arrays.asList(FieldConstants.redReef1.toPose2d(),
              FieldConstants.redCoralLoadingTop.toPose2d(), FieldConstants.redReef1.toPose2d(),
              FieldConstants.redCoralLoadingTop.toPose2d()));

      // Middle of blue reef to top intake auto routine
      public static final ArrayList<Pose2d> redMiddleToTopIntakePositions = new ArrayList<Pose2d>(
        Arrays.asList(FieldConstants.redReef3.toPose2d(),
            FieldConstants.redCoralLoadingTop.toPose2d(), FieldConstants.redReef1.toPose2d(),
            FieldConstants.redCoralLoadingTop.toPose2d()));

      // Middle of blue reef to bottom intake auto routine
      public static final ArrayList<Pose2d> redMiddleToBottomIntakePositions = new ArrayList<Pose2d>(
        Arrays.asList(FieldConstants.redReef3.toPose2d(),
            FieldConstants.redCoralLoadingBottom.toPose2d(), FieldConstants.redReef5.toPose2d(),
            FieldConstants.redCoralLoadingBottom.toPose2d()));

      // Bottom of blue reef to intake auto routine
      public static final ArrayList<Pose2d> redBottomToIntakePositions = new ArrayList<Pose2d>(
        Arrays.asList(FieldConstants.redReef5.toPose2d(),
            FieldConstants.redCoralLoadingBottom.toPose2d(), FieldConstants.redReef5.toPose2d(),
            FieldConstants.redCoralLoadingBottom.toPose2d()));
    }
  }

  public static final class PhotonvisionConstants {

    public static final List<PhotonUnit> photonUnits = Arrays.asList(
      new PhotonUnit("FrontCenterCamera",
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        new Transform3d(new Pose3d(),
          new Pose3d(new Translation3d(Units.inchesToMeters(6.7),
              Units.inchesToMeters(11), Units.inchesToMeters(7.1875)),
            new Rotation3d(0, 0, Units.degreesToRadians(0)))),
        FieldConstants.fieldLayout));
      // new PhotonUnit("BackRightCamera",
      //   PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
      //   new Transform3d(new Pose3d(),
      //     new Pose3d(new Translation3d(Units.inchesToMeters(-12.5),
      //         Units.inchesToMeters(-7), Units.inchesToMeters(7.1875)),
      //       new Rotation3d(0, 0, Units.degreesToRadians(215)))),
      //   FieldConstants.fieldLayout),
      // new PhotonUnit("BackLeftCamera",
      //   PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
      //   new Transform3d(new Pose3d(),
      //     new Pose3d(new Translation3d(Units.inchesToMeters(-12.5),
      //           Units.inchesToMeters(7), Units.inchesToMeters(7.1875)),
      //         new Rotation3d(0, 0, Units.degreesToRadians(145)))),
      //   FieldConstants.fieldLayout));

    public static final double poseEstimatorAmbiguityScaleFactor = 2;
    public static final double photonUnitAmbiguityCutoff = 0.1;
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
      public static final double defaultPositionBuffer = 0.1;
    }

    public static final class FollowConstants {
      public static final SplineInterpolator defaultInterpolator = new LinearInterpolator(); // Could change to cubic
      public static final double maxSpeed = 2.8;
      public static final double maxCentrifugalAcceleration = 2;
      public static final double maxAccelAfterTask = 1.5;
      public static final boolean interpolateFromStart = true;

      public static final double staticThetaVelocity = 0.12;

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
        ProfiledPIDController thetaController =
          new ProfiledPIDController(1.2, 0, 0, new Constraints(50 * Math.PI, 50 * Math.PI));
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
        return 5 * remainingLength + 0.2;
      }
    }
  }

  public static final class DifferentialArm {
    // public static final double encoderOffset = 0.5;
    public static final double encoderRange = 0.4;
    public static final double manualThreshold = 0.1;

    public static final double currentDifferenceThreshold = 15;
    public static final double positionDelta = 0.05;
    public static final double filterTimeConstant = 0.5;

    public static final long deployTime = 850; // Milliseconds
    public static final long rampTime = 250;
    public static final double defaultGravityCompensation = 0.1;

    public static final double manualMovementPerSecond = 0.15;

    public static final class MotorIDs {
      public static final int leftID = 13;
      public static final int rightID = 14;
    }

    public static final class PID {
      public static final double P = 1.8;
      public static final double I = 0;
      public static final double D = 0.02;
    }
  }

  // Important setpoints for the claw
  public static final class Setpoints {
    // Constant intake power (just invert to deposit)
    public static final double intakePower = 0.7;

    // Angles for the different pipe deposit levels, and an angle for intake
    public static final class LevelAngles {
      public static final double Start = 0.63;
      public static final double Intake = 0.524;
      public static final double Store = 0.7;
      public static final double L1 = 0.48;
      public static final double L23 = 0.45;//0.48;
      public static final double L4 = 0.367;
      public static final double Transit = 0.7;
      public static final double Algae = 0.554;
    }
  }
}

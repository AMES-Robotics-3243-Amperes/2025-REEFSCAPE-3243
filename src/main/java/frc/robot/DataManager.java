package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.PhotonvisionConstants;
import frc.robot.Constants.SwerveConstants.ChassisKinematics;
import frc.robot.Constants.ElevatorPositions;
import frc.robot.PhotonUnit.Measurement;
import frc.robot.subsystems.SubsystemElevator;
import frc.robot.subsystems.SubsystemSwerveDrivetrain;
import frc.robot.utility.AHRS_IMU;
import frc.robot.utility.IMU;

public class DataManager {
  /** Singleton instance */
  private static DataManager instance;

  // PROPOSAL: Deprecate or remove this method.
  // REASON: Maintaining Dependency Inversion (the D in SOLID) to avoid
  // DataManager being used before it is configured, or used in places
  // another system would be better. The DataManager instance should always
  // be accessed through dependency injection (passing a thing into a constructor)
  // to this end.
  // H!
  public static DataManager instance() {
    return instance;
  }

  /**
   * An {@link Entry} that is automatically updated upon a call to
   * {@link #update DataManager.instance().update}.
   */
  public abstract class DataManagerEntry<T> extends Entry<T> {
    public DataManagerEntry() {
      entries.add(this);
    }
  }

  public class RobotPosition extends DataManagerEntry<Pose2d> {
    /** used to combine vision and odometry data */
    private SwerveDrivePoseEstimator poseEstimator;

    private double previousTimestamp = 0;
    private Translation2d previousPosition = new Translation2d();
    private Translation2d velocity = new Translation2d();

    private SubsystemSwerveDrivetrain subsystemSwerveDrivetrain;
    private List<PhotonUnit> photonUnits = new ArrayList<PhotonUnit>();
    private IMU imu = new AHRS_IMU();

    private Field2d field2d = new Field2d();

    public RobotPosition(RobotContainer robotContainer) {
      subsystemSwerveDrivetrain = robotContainer.subsystemSwerveDrivetrain;
      this.photonUnits = new ArrayList<PhotonUnit>(PhotonvisionConstants.photonUnits);

      poseEstimator = new SwerveDrivePoseEstimator(ChassisKinematics.kDriveKinematics, imu.getRotation(),
          subsystemSwerveDrivetrain.getModulePositions(), new Pose2d());
    }

    public void update() {
      poseEstimator.update(imu.getRotation(), subsystemSwerveDrivetrain.getModulePositions());

      for (var unit : photonUnits) {
        for (Measurement measurement : unit.getMeasurement()) {
          Pose3d threeDPose = new Pose3d(poseEstimator.getEstimatedPosition());
          double distance = threeDPose.plus(unit.getRobotToCamera()).toPose2d().getTranslation()
              .getDistance(measurement.targetPosition);

          if (distance < PhotonvisionConstants.photonUnitMinDistance 
              || velocity.getNorm() > PhotonvisionConstants.photonUnitVelocityCutoff)
            continue;

          poseEstimator.addVisionMeasurement(measurement.pose, measurement.timestampSeconds,
              measurement.ambiguity.times(PhotonvisionConstants.poseEstimatorAmbiguityScaleFactor
                  * (distance + velocity.getNorm() + 1)));
        }
      }

      double currentTimestamp = MathSharedStore.getTimestamp();
      Translation2d currentPosition = poseEstimator.getEstimatedPosition().getTranslation();
      velocity = currentPosition.minus(previousPosition).times(currentTimestamp - previousTimestamp);
      previousTimestamp = currentTimestamp;
      previousPosition = currentPosition;

      field2d.setRobotPose(get());
      SmartDashboard.putData(field2d);
    }

    public void set(Pose2d newPose) {
      poseEstimator.resetPosition(imu.getRotationModulus(), subsystemSwerveDrivetrain.getModulePositions(), newPose);
    }

    public Pose2d get() {
      return poseEstimator.getEstimatedPosition();
    }
  }

  public static enum ElevatorSetpoint {
    Between(null),
    L1(ElevatorPositions.L1),
    L2(ElevatorPositions.L2),
    L3(ElevatorPositions.L3),
    L4(ElevatorPositions.L4);

    public final Double position;

    private ElevatorSetpoint(Double position) {
      this.position = position;
    }
  }

  public static class ElevatorPositionData {
    // H! TODO Put constants in Constants.java
    public static final double DELTAP = 0.15;
    public static final double DELTAV = 0.05;

    public final double exactPos;
    public final double exactVel;
    public final ElevatorSetpoint setpoint;

    public ElevatorPositionData(double position, double velocity) {
      this.exactPos = position;
      this.exactVel = velocity;

      ElevatorSetpoint generalPosToBeSet = ElevatorSetpoint.Between;
      double minPositionDif = DELTAP; // Position dif must always be than DELTAP
      for (ElevatorSetpoint value : ElevatorSetpoint.values()) {
        if (value.position == null) {
          continue;
        } // Skip over ElevatorSetpoint.Between
        if (Math.abs(velocity) < DELTAV && Math.abs(value.position - position) < minPositionDif) {
          generalPosToBeSet = value;
          minPositionDif = Math.abs(value.position - position);
          // Continue in case a future setpoint is closer.
        }
      }

      setpoint = generalPosToBeSet;
    }
  }

  public class ElevatorPosition extends DataManagerEntry<ElevatorPositionData> {
    private SubsystemElevator elevator;

    public ElevatorPosition(RobotContainer robotContainer) {
      elevator = robotContainer.subsystemElevator;
    }

    @Override
    public ElevatorPositionData get() {
      return new ElevatorPositionData(elevator.getPosition(), elevator.getVelocity());
    }
  }

  @SuppressWarnings("rawtypes")
  private ArrayList<DataManagerEntry> entries = new ArrayList<>();

  public DataManagerEntry<Pose2d> robotPosition;
  public ElevatorPosition elevatorPosition;

  public void update() {
    entries.forEach(entry -> entry.update());
  }

  /**
   * Constructs the singleton from member variables of a RobotContainer.
   * Should be called ASAP, preferably in the constructor for RobotContainer.
   */
  public DataManager(RobotContainer robotContainer) {
    robotPosition = new RobotPosition(robotContainer);
    elevatorPosition = new ElevatorPosition(robotContainer);

    instance = this;
  }
}

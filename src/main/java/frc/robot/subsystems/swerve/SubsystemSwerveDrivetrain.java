package frc.robot.subsystems.swerve;

import java.util.Optional;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DataManager;
import frc.robot.Constants.SwerveConstants.DriveTrainConstants;
import frc.robot.Constants.SwerveConstants.ModuleConstants;
import frc.robot.subsystems.swerve.control.SwerveControlRequest;
import frc.robot.subsystems.swerve.control.SwerveControlRequest.ControlOutput;
import frc.robot.subsystems.swerve.modules.SwerveModule;
import frc.robot.subsystems.swerve.modules.ThriftyModule;

public class SubsystemSwerveDrivetrain extends SubsystemBase {
  private final SwerveModule m_frontLeft = new ThriftyModule(
      DriveTrainConstants.IDs.kFrontLeftDrivingCanId,
      DriveTrainConstants.IDs.kFrontLeftTurningCanId, 0, DriveTrainConstants.ModuleOffsets.kFrontLeftOffset);

  private final SwerveModule m_frontRight = new ThriftyModule(
      DriveTrainConstants.IDs.kFrontRightDrivingCanId,
      DriveTrainConstants.IDs.kFrontRightTurningCanId, 1, DriveTrainConstants.ModuleOffsets.kFrontRightOffset);

  private final SwerveModule m_rearLeft = new ThriftyModule(
      DriveTrainConstants.IDs.kRearLeftDrivingCanId,
      DriveTrainConstants.IDs.kRearLeftTurningCanId, 2, DriveTrainConstants.ModuleOffsets.kBackLeftOffset);

  private final SwerveModule m_rearRight = new ThriftyModule(
      DriveTrainConstants.IDs.kRearRightDrivingCanId,
      DriveTrainConstants.IDs.kRearRightTurningCanId, 3, DriveTrainConstants.ModuleOffsets.kBackRightOffset);

  private Optional<SwerveControlRequest> request = Optional.empty();
  private Translation2d currentVelocity = new Translation2d();
  private Timer accelerationTimer = new Timer();

  public SubsystemSwerveDrivetrain() {
  }

  public void setControlRequest(SwerveControlRequest request) {
    this.request = Optional.ofNullable(request);
  }

  public void setControlRequest(Optional<SwerveControlRequest> request) {
    this.request = request;
  }

  /**
   * Set the swerve modules' desired states
   *
   * @param desiredStates the desired {@link SwerveModuleState}s
   * 
   * @author :3
   */
  private void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, ModuleConstants.kMaxObtainableModuleSpeed);

    // :3 set the desired states
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /**
   * Used for pose estimation.
   * 
   * @author :3
   * @return the positions of the swerve modules
   */
  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        m_frontLeft.getPosition(), m_frontRight.getPosition(),
        m_rearLeft.getPosition(), m_rearRight.getPosition()
    };
  }

  /**
   * Used for pose estimation.
   * 
   * @author :3
   * @return the positions of the swerve modules
   */
  public SwerveModulePosition[] getAbsoluteModulePositions() {
    return new SwerveModulePosition[] {
        m_frontLeft.getAbsolutePosition(), m_frontRight.getAbsolutePosition(),
        m_rearLeft.getAbsolutePosition(), m_rearRight.getAbsolutePosition()
    };
  }

  @Override
  public void periodic() {
    if (request.isPresent()) {
      accelerationTimer.stop();

      ControlOutput output = request.get().getOutput(currentVelocity, accelerationTimer.get());
      setModuleStates(output.getStates());

      ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(output.getRobotRelativeSpeeds(),
          DataManager.instance().robotPosition.get().getRotation());
      currentVelocity = new Translation2d(fieldRelativeSpeeds.vxMetersPerSecond, fieldRelativeSpeeds.vyMetersPerSecond);

      accelerationTimer.reset();
      accelerationTimer.start();
    }

    m_frontLeft.update();
    m_frontRight.update();
    m_rearLeft.update();
    m_rearRight.update();
  }

  // #################
  // INTEGRATION TESTS
  // #################

  // @Override
  // public Test[] getTests() {
  // return tests;
  // }

  // private boolean moduleRotationTest2Done = false;
  // private Future<Boolean> moduleRotationTestUserQuestion;
  // @SuppressWarnings("unchecked")
  // private Test[] tests = {
  // new TestUtil.MultiphaseTest(
  // new Runnable[] { this::moduleRotationTest1, this::moduleRotationTest2,
  // this::moduleRotationTest3 },
  // (Supplier<Boolean>[]) new Supplier[] { () -> true, () ->
  // moduleRotationTest2Done, () -> true },
  // "Module Rotation Test")
  // };

  // /**
  // * Ensures the {@link setModuleRotations} method functions properly, according
  // * to user viewing, and encoders.
  // *
  // * Part 1.
  // */
  // private void moduleRotationTest1() {
  // setModuleRotations(new Rotation2d[] {
  // new Rotation2d(0),
  // new Rotation2d(0),
  // new Rotation2d(0),
  // new Rotation2d(0)
  // });

  // moduleRotationTest2Done = false;
  // moduleRotationTestUserQuestion = TestUtil.askUserBool("Are all wheels
  // pointing forward?");
  // }

  // private void moduleRotationTest2() {
  // if (moduleRotationTestUserQuestion.isDone()) {
  // boolean areWheelsCorrect;
  // try {
  // areWheelsCorrect = moduleRotationTestUserQuestion.get();
  // } catch (Exception e) {
  // throw new AssertionError(e);
  // }

  // TestUtil.assertBool(areWheelsCorrect, "Wheels did not point forward when
  // given that command.");

  // moduleRotationTest2Done = true;
  // }
  // }

  // private void moduleRotationTest3() {
  // SwerveModulePosition[] positions = getModulePositions();

  // for (SwerveModulePosition position : positions) {
  // double angleDif = position.angle.minus(new Rotation2d()).getDegrees() % 360;
  // if (angleDif > 10 || angleDif < -10) {
  // throw new AssertionError("Swerve module encoder did not show forward facing
  // direction.");
  // }
  // }
  // }
}

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants.kSwerve;
import frc.robot.Constants.kVision;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Swerve;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionTest extends CommandBase {
	private final Swerve swerve;
	private final PhotonCamera leftCamera;
	private final PhotonCamera rightCamera;
	private final PoseEstimator poseEstimator;

	// PID controlers
	private static final TrapezoidProfile.Constraints turnConstraints = new TrapezoidProfile.Constraints(3, 2);
	final double turnP = 2;
	final double turnD = 0.0;
	private final ProfiledPIDController turnController = new ProfiledPIDController(turnP, 0, turnD, turnConstraints);

	private static final TrapezoidProfile.Constraints xyConstraints = new TrapezoidProfile.Constraints(3, 2);
	final double xyP = 3;
	final double xyD = 0.0;
	private final ProfiledPIDController xController = new ProfiledPIDController(xyP, 0, xyD, xyConstraints);
	private final ProfiledPIDController yController = new ProfiledPIDController(xyP, 0, xyD, xyConstraints);

	private PhotonTrackedTarget lastTarget;

	private static final Transform3d tagToGoal = new Transform3d(
			new Translation3d(1.5, 0.0, 0.0),
			new Rotation3d(0.0, 0.0, Math.PI));
	public int tagId = 0;

	public VisionTest(Swerve swerve, PhotonCamera rightCamera, PhotonCamera leftCamera, PoseEstimator poseEstimator,
			int tagId) {
		this.swerve = swerve;
		this.leftCamera = leftCamera;// new PhotonCamera(Vision.leftCameraName);
		this.rightCamera = rightCamera;// new PhotonCamera(Vision.leftCameraName);
		this.poseEstimator = poseEstimator;
		this.tagId = tagId;

		xController.setTolerance(0.2);
		yController.setTolerance(0.2);
		turnController.setTolerance(Units.degreesToRadians(3));
		turnController.enableContinuousInput(-Math.PI, Math.PI);

		addRequirements(swerve);
	}

	@Override
	public void initialize() {
		lastTarget = null;
		Pose2d robotPose = poseEstimator.getCurrentPose();
		turnController.reset(robotPose.getRotation().getRadians());
		xController.reset(robotPose.getX());
		yController.reset(robotPose.getY());
	}

	@Override
	public void execute() {
		Pose2d robotPose2d = poseEstimator.getCurrentPose();
		Pose3d robotPose = new Pose3d(
				robotPose2d.getX(),
				robotPose2d.getY(),
				0.0,
				new Rotation3d(swerve.getRoll().getRadians(), swerve.getPitch().getRadians(),
						robotPose2d.getRotation().getRadians()));

		PhotonPipelineResult photonRes = leftCamera.getLatestResult();
		if (photonRes.hasTargets()) {
			// Find the tag we want to chase
			Optional<PhotonTrackedTarget> targetOpt = photonRes.getTargets().stream()
					.filter(target -> target.getFiducialId() == tagId)
					.filter(target -> !target.equals(lastTarget) && target.getPoseAmbiguity() <= .2
							&& target.getPoseAmbiguity() != -1)
					.findFirst();
			if (targetOpt.isPresent()) {
				PhotonTrackedTarget target = targetOpt.get();
				// This is new target data, so recalculate the goal
				lastTarget = target;

				// Transform the robot's pose to find the camera's pose
				Pose3d cameraPose = robotPose.transformBy(kVision.leftCamRelative);

				// Trasnform the camera's pose to the target's pose
				Transform3d camToTarget = target.getBestCameraToTarget();
				Pose3d targetPose = cameraPose.transformBy(camToTarget);

				// Transform the tag's pose to set our goal
				Pose2d goalPose = targetPose.transformBy(tagToGoal).toPose2d();

				// Drive
				xController.setGoal(goalPose.getX());
				yController.setGoal(goalPose.getY());
				turnController.setGoal(goalPose.getRotation().getRadians());
			}
		}

		if (lastTarget != null) {
			double xSpeed = xController.calculate(robotPose.getX());
			if (xController.atGoal()) {
				xSpeed = 0;
			}

			double ySpeed = yController.calculate(robotPose.getY());
			if (yController.atGoal()) {
				ySpeed = 0;
			}

			double turnSpeed = turnController.calculate(robotPose2d.getRotation().getRadians());
			if (turnController.atGoal()) {
				turnSpeed = 0;
			}

			swerve.drive(
					new Translation2d(xSpeed, ySpeed),
					turnSpeed,
					true,
					kSwerve.openLoop);

			/*
			 * PhotonPipelineResult leftResult = leftCamera.getLatestResult();
			 * PhotonPipelineResult rightResult = leftCamera.getLatestResult();
			 *
			 * List<PhotonTrackedTarget> leftTargets = leftResult.getTargets();
			 * List<PhotonTrackedTarget> rightTargets = rightResult.getTargets();
			 *
			 * boolean hasLeftTarget = leftResult.hasTargets();
			 * boolean hasRightTarget = rightResult.hasTargets();
			 *
			 * ArrayList<Integer> leftTargetsFiducialId = new ArrayList<Integer>();
			 * if (hasLeftTarget) {
			 * for (PhotonTrackedTarget leftTarget : leftTargets) {
			 * leftTargetsFiducialId.add(leftTarget.getFiducialId());
			 * }
			 * }
			 * ArrayList<Integer> rightTargetsFiducialId = new ArrayList<Integer>();
			 * if (hasRightTarget) {
			 * for (PhotonTrackedTarget rightTarget : rightTargets) {
			 * rightTargetsFiducialId.add(rightTarget.getFiducialId());
			 * }
			 * }
			 *
			 * double speed = 0;
			 * Transform3d test = rightTargets.get(0).getBestCameraToTarget();
			 *
			 * //PhotonUtils.
			 * // result.getBestTarget().getFiducialId()
			 * if (hasLeftTarget)
			 * speed = -turnController.calculate(leftTargets.get(0).getYaw(), 0);
			 *
			 * swerve.drive(new Translation2d(), speed * 0.75, false, SwerveConst.openLoop);
			 */
		}
	}
}

/*
 * public void execute() {
 * var robotPose2d = poseProvider.get();
 * var robotPose =
 * new Pose3d(
 * robotPose2d.getX(),
 * robotPose2d.getY(),
 * 0.0,
 * new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getRadians()));
 *
 * var photonRes = photonCamera.getLatestResult();
 * if (photonRes.hasTargets()) {
 * // Find the tag we want to chase
 * var targetOpt = photonRes.getTargets().stream()
 * .filter(t -> t.getFiducialId() == TAG_TO_CHASE)
 * .filter(t -> !t.equals(lastTarget) && t.getPoseAmbiguity() <= .2 &&
 * t.getPoseAmbiguity() != -1)
 * .findFirst();
 * if (targetOpt.isPresent()) {
 * var target = targetOpt.get();
 * // This is new target data, so recalculate the goal
 * lastTarget = target;
 *
 * // Transform the robot's pose to find the camera's pose
 * var cameraPose = robotPose.transformBy(ROBOT_TO_CAMERA);
 *
 * // Trasnform the camera's pose to the target's pose
 * var camToTarget = target.getBestCameraToTarget();
 * var targetPose = cameraPose.transformBy(camToTarget);
 *
 * // Transform the tag's pose to set our goal
 * var goalPose = targetPose.transformBy(TAG_TO_GOAL).toPose2d();
 *
 * // Drive
 * xController.setGoal(goalPose.getX());
 * yController.setGoal(goalPose.getY());
 * omegaController.setGoal(goalPose.getRotation().getRadians());
 * }
 * }
 *
 * if (lastTarget == null) {
 * // No target has been visible
 * drivetrainSubsystem.stop();
 * } else {
 * // Drive to the target
 * var xSpeed = xController.calculate(robotPose.getX());
 * if (xController.atGoal()) {
 * xSpeed = 0;
 * }
 *
 * var ySpeed = yController.calculate(robotPose.getY());
 * if (yController.atGoal()) {
 * ySpeed = 0;
 * }
 *
 * var omegaSpeed =
 * omegaController.calculate(robotPose2d.getRotation().getRadians());
 * if (omegaController.atGoal()) {
 * omegaSpeed = 0;
 * }
 *
 * drivetrainSubsystem.drive(
 * ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed,
 * robotPose2d.getRotation()));
 * }
 * }
 *
 * @Override
 * public void end(boolean interrupted) {
 * drivetrainSubsystem.stop();
 * }
 *
 * }
 */

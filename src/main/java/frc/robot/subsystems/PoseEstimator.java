package frc.robot.subsystems;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static frc.robot.Constants.kVision.*;

import java.util.Collections;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kSwerve;

public class PoseEstimator extends SubsystemBase {

	private final PhotonCamera photonCamera;
	private final Swerve swerve;

	// Ordered list of target poses by ID (WPILib is adding some functionality for
	// this)
	private static final List<Pose3d> targetPoses = Collections.unmodifiableList(List.of(
			new Pose3d(3.0, 1.165, 0.287 + 0.165, new Rotation3d(0, 0, degreesToRadians(180.0))),
			new Pose3d(3.0, 0.0, 0.287 + .165, new Rotation3d(0, 0, degreesToRadians(180.0)))));

	private final SwerveDrivePoseEstimator swervePoseEstimator;

	private final Field2d field2d = new Field2d();

	private double previousPipelineTimestamp = 0;

	/**
	 *
	 * @param photonCamera this is left camera
	 * @param swerve
	 */
	public PoseEstimator(PhotonCamera photonCamera, Swerve swerve) {
		this.photonCamera = photonCamera;
		this.swerve = swerve;

		ShuffleboardTab tab = Shuffleboard.getTab("Vision");
		swervePoseEstimator = new SwerveDrivePoseEstimator(kSwerve.swerveKinematics, swerve.getYaw(),
				swerve.getPos(), new Pose2d());

		tab.addString("Pose", this::getFomattedPose).withPosition(0, 0).withSize(2, 0);
		tab.add("Field", field2d).withPosition(2, 0).withSize(6, 4);
	}

	@Override
	public void periodic() {
		// Update pose estimator with the best visible target
		PhotonPipelineResult pipelineResult = photonCamera.getLatestResult();
		double resultTimestamp = pipelineResult.getTimestampSeconds();
		if (resultTimestamp != previousPipelineTimestamp && pipelineResult.hasTargets()) {
			previousPipelineTimestamp = resultTimestamp;
			PhotonTrackedTarget target = pipelineResult.getBestTarget();
			int fiducialId = target.getFiducialId();
			if (target.getPoseAmbiguity() <= .2 && fiducialId >= 0 && fiducialId < targetPoses.size()) {
				Pose3d targetPose = targetPoses.get(fiducialId);
				Transform3d camToTarget = target.getBestCameraToTarget();
				Pose3d camPose = targetPose.transformBy(camToTarget.inverse());

				Pose3d visionMeasurement = camPose.transformBy(leftCamRelative);
				swervePoseEstimator.addVisionMeasurement(visionMeasurement.toPose2d(), resultTimestamp);
			}
		}
		// Update pose estimator with drivetrain sensors
		swervePoseEstimator.update(
				swerve.getYaw(),
				swerve.getPos());

		field2d.setRobotPose(getCurrentPose());
	}

	private String getFomattedPose() {
		var pose = getCurrentPose();
		return String.format("(%.2f, %.2f) %.2f degrees",
				pose.getX(),
				pose.getY(),
				pose.getRotation().getDegrees());
	}

	public Pose2d getCurrentPose() {
		return swervePoseEstimator.getEstimatedPosition();
	}

	/**
	 * Resets the current pose to the specified pose. This should ONLY be called
	 * when the robot's position on the field is known, like at the beginning of
	 * a match.
	 *
	 * @param newPose new pose
	 */
	public void setCurrentPose(Pose2d newPose) {
		swervePoseEstimator.resetPosition(
				swerve.getYaw(),
				swerve.getPos(),
				newPose);
	}

	/**
	 * Resets the position on the field to 0,0 0-degrees, with forward being
	 * downfield. This resets
	 * what "forward" is for field oriented driving.
	 */
	public void resetFieldPosition() {
		setCurrentPose(new Pose2d());
	}

}

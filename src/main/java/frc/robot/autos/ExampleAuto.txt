package frc.robot.autos;

import frc.robot.Constants.kAuto;
import frc.robot.Constants.kSwerve;
import frc.robot.subsystems.Swerve;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class ExampleAuto extends SequentialCommandGroup {
	public ExampleAuto(Swerve swerve) {
		// An example trajectory to follow. All units in meters.
		Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
				// Start at the origin facing the +X direction
				new Pose2d(0, 0, new Rotation2d(0)),
				// Pass through these two interior waypoints, making an 's' curve path
				List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
				// End 3 meters straight ahead of where we started, facing forward
				new Pose2d(3, 0, new Rotation2d(0)),
				kAuto.config);

		var thetaController = new ProfiledPIDController(
				kAuto.kPThetaController, 0, 0, kAuto.kThetaControllerConstraints);
		thetaController.enableContinuousInput(-Math.PI, Math.PI);

		SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
				exampleTrajectory,
				swerve::getPose,
				kSwerve.swerveKinematics,
				new PIDController(kAuto.kPXController, 0, 0),
				new PIDController(kAuto.kPYController, 0, 0),
				thetaController,
				swerve::setModuleStates,
				swerve);

		addCommands(
				swerve.runOnce(() -> swerve.resetOdometry(exampleTrajectory.getInitialPose())),
				swerveControllerCommand);
	}
}

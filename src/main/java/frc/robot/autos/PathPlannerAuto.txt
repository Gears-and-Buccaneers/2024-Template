package frc.robot.autos;

import frc.robot.Constants.kAuto;
import frc.robot.Constants.kSwerve;
import frc.robot.subsystems.Swerve;

import com.pathplanner.lib.PathPlanner;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PrintCommand;

import com.pathplanner.lib.auto.SwerveAutoBuilder;

public class PathPlannerAuto {
	private SwerveAutoBuilder autoBuilder;

	public CommandBase get(String name, Swerve swerve, HashMap<String, Command> eventMap) {
		autoBuilder = new SwerveAutoBuilder(
				swerve::getPose, // Pose2d supplier
				swerve::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
				kSwerve.swerveKinematics, // SwerveDriveKinematics
				kAuto.translation, // PID constants to correct for translation error (used to create the X
				// and Y PID controllers)
				kAuto.rotation, // PID constants to correct for rotation error (used to create the
				// rotation controller)
				swerve::setModuleStates, // Module states consumer used to output to the drive subsystem
				eventMap,
				swerve // The drive subsystem. Used to properly set the requirements of path following
		// commands
		);
		return autoBuilder.fullAuto(PathPlanner.loadPathGroup(name, kAuto.constraints));
	}

}

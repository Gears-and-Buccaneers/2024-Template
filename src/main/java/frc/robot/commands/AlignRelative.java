package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.Camera;
import frc.robot.Constants.kSwerve;
import frc.robot.subsystems.Swerve;

public class AlignRelative extends CommandBase {
	private final Swerve swerve;

	final PIDController xController = new PIDController(3, 0, 3);
	final PIDController yController = new PIDController(3, 0, 3);
	final PIDController rController = new PIDController(2, 0, 0);

	final int tagId;

	boolean isDone = false;

	public AlignRelative(Swerve swerve, int tagId, Pose2d target) {
		this.swerve = swerve;
		this.tagId = tagId;

		xController.setSetpoint(target.getX());
		yController.setSetpoint(target.getY());
		rController.setSetpoint(target.getRotation().getRadians());

		xController.setTolerance(0.2);
		yController.setTolerance(0.2);
		rController.setTolerance(Units.degreesToRadians(3));

		rController.enableContinuousInput(-Math.PI, Math.PI);

		addRequirements(swerve);
	}

	@Override
	public void execute() {
		Transform3d tagRelative = Camera.tags().get(tagId);

		if (tagRelative == null) {
			isDone = true;
			return;
		}

		swerve.drive(new Translation2d(
				xController.calculate(tagRelative.getX()),
				yController.calculate(tagRelative.getY())),
				rController.calculate(tagRelative.getRotation().getZ()), true, kSwerve.openLoop);
	}

	@Override
	public boolean isFinished() {
		return isDone || (xController.atSetpoint() && yController.atSetpoint() && rController.atSetpoint());
	}
}

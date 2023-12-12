package frc.robot.commands;

import frc.robot.Constants.kSwerve;
import frc.robot.Constants.Controls;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleopSwerve extends CommandBase {
	private Swerve swerve;

	public TeleopSwerve(Swerve swerve) {
		this.swerve = swerve;
		addRequirements(swerve);
	}

	@Override
	public void execute() {
		swerve.drive(
				new Translation2d(
						Controls.driver.LS_Y.get(),
						Controls.driver.LS_X.get()).times(Controls.driver.LT_S.get() > 0.5 ? 0.25 : 1)
						.times(Controls.driver.A.getAsBoolean() ? 1 / kSwerve.maxSpeed : 1),
				(Controls.driver.LT_S.get() > 0.5 ? Controls.driver.RS_X.get() * .25 : Controls.driver.RS_X.get())
						* kSwerve.maxAngularVelocity,
				!Controls.driver.LB.getAsBoolean(),
				kSwerve.openLoop);
	}
}

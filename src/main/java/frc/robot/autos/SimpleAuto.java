package frc.robot.autos;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.kSwerve;
import frc.robot.Constants.kBoom.Level;
import frc.robot.commands.AutoBalance;
import frc.robot.subsystems.Boom;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Swerve;

public class SimpleAuto {
	public static CommandBase dropPiece(Boom boom, Gripper gripper, Level level) {
		return boom.setTo(Level.INTAKE).andThen(
				new WaitCommand(.7),
				boom.setTo(level),
				gripper.runOnce(gripper::toggle),
				new WaitCommand(.5),
				gripper.runOnce(gripper::toggle),
				new WaitCommand(.5),
				boom.setTo(Level.BOTTOM));
	}

	public static CommandBase autoBalance(Swerve swerve) {
		return swerve.run(() -> swerve.drive(
				new Translation2d(0.25, 0), 0, false, kSwerve.openLoop)) // drives backward at 25% speed
				.until(
						() -> Math.abs(MathUtil.inputModulus(swerve.getPitch().getDegrees(), -180, 180)) > 10)
				.withTimeout(2).andThen(
						new AutoBalance(swerve), // then it runs the serve auto balace wich makes the robot leval
						lockIn(swerve) // then locks in the robot
				);
	}

	public static CommandBase autoBalanceStep(Swerve swerve) {
		return swerve.run(() -> swerve.drive(
				new Translation2d(0.25, 0), 0, false, kSwerve.openLoop)) // drives backward at 25% speed
				.until(
						() -> Math.abs(MathUtil.inputModulus(swerve.getPitch().getDegrees(), -180, 180)) > 10)
				.withTimeout(2).andThen(
						balanceStep(swerve), // then it runs the serve auto balace wich makes the robot leval
						lockIn(swerve) // then locks in the robot
				);
	}

	public static CommandBase lockIn(Swerve swerve) {
		return swerve.runOnce(() -> { // locks the robot in so it cant move.
			swerve.drive(new Translation2d(), 0.01, true, true);
			swerve.drive(new Translation2d(), 0, true, true);
		});
	}

	public static CommandBase exitCommunity(Swerve swerve) {
		return swerve.run(
				() -> swerve.drive(new Translation2d(-0.5, 0), 0.0, true, kSwerve.openLoop)).withTimeout(2.4);
	}

	public static CommandBase balanceStep(Swerve swerve) {
		ShuffleboardTab config = Shuffleboard.getTab("Autos");

		GenericEntry target = config.add("Step-balance pitch threshold", 3.0).getEntry();
		GenericEntry speed = config.add("Step-balance speed", 0.025).getEntry();
		GenericEntry driveTime = config.add("Step-balance drive-time", 0.2).getEntry();
		GenericEntry waitTime = config.add("Step-balance wait-time", 0.2).getEntry();
		GenericEntry lock = config.add("Lock-in", true).getEntry();

		return swerve.run(() -> swerve.drive(new Translation2d(speed.getDouble(0.025), 0), 0, false, kSwerve.openLoop))
				.andThen(
						new WaitCommand(driveTime.getDouble(0.2)),
						lockIn(swerve).unless(() -> !lock.getBoolean(true)),
						new WaitCommand(waitTime.getDouble(0.2)),
						swerve.run(() -> {
							if (Math.abs(swerve.getPitch().getDegrees()) <= target.getDouble(3.0))
								balanceStep(swerve).schedule();
						}));
	}

	// public static CommandBase balanceStep2(Swerve swerve) {
	// ShuffleboardTab config = Shuffleboard.getTab("Autos");

	// double last = 0;

	// GenericEntry target = config.add("Step-balance pitch threshold",
	// 3.0).getEntry();
	// GenericEntry speed = config.add("Step-balance speed", 0.025).getEntry();

	// return swerve.run(() -> last = swerve.getPitch().getDegrees())
	// .andThen(
	// () -> swerve.drive(new Translation2d(speed.getDouble(0.01), 0), 0, false,
	// kSwerve.openLoop)
	// .until(),
	// lockIn(swerve),
	// new WaitCommand(waitTime.getDouble(0.2)),
	// swerve.run(() -> {
	// if (Math.abs(swerve.getPitch().getDegrees()) <= target.getDouble(3.0))
	// balanceStep(swerve).schedule();
	// }));
	// }

}

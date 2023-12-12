package frc.robot;

// import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.Controls;
import frc.robot.Constants.kBoom.Level;

import frc.robot.autos.*;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

public class Robot extends TimedRobot {
	// robot SUBSYSTEMS
	private final Swerve swerve = new Swerve();
	private final Boom boom = new Boom();
	private final Gripper gripper = new Gripper();

	// AUTOS
	private SendableChooser<Command> chooser1 = new SendableChooser<>();
	private SendableChooser<Command> chooser2 = new SendableChooser<>();
	private Command autonomousCommand;

	@Override
	public void robotInit() {
		// INITIALIZATION
		swerve.setDefaultCommand(new TeleopSwerve(swerve));
		// CameraServer.startAutomaticCapture();
		SmartDashboard.putData(CommandScheduler.getInstance());

		// CONTROLS
		// SmartDashboard.putData("zero gyro", swerve.run(swerve::zeroGyro));
		// SmartDashboard.putData("zero Boom encoder", boom.runOnce(boom::zeroEncoder));

		Controls.driver.BACK.whileTrue(swerve.runOnce(swerve::zeroGyro));
		Controls.driver.RB.whileTrue(gripper.runOnce(gripper::toggle));
		Controls.driver.Y.whileTrue(gripper.runOnce(gripper::toggle));

		// Controls.driver.A.whileTrue(SimpleAuto.autoBalance(swerve));

		Controls.driver.DOWN.onTrue(boom.setTo(Level.INTAKE));
		Controls.driver.LEFT.onTrue(boom.setTo(Level.BOTTOM));
		Controls.driver.UP.onTrue(boom.setTo(Level.MIDDLE));
		// Controls.driver.RIGHT.onTrue(boom.setTo(Level.TOP));

		// AUTOS
		chooser1.setDefaultOption("Bottom",
				SimpleAuto.dropPiece(boom, gripper, Level.BOTTOM));
		chooser1.addOption("Top",
				SimpleAuto.dropPiece(boom, gripper, Level.TOP));
		chooser1.addOption("Middle",
				SimpleAuto.dropPiece(boom, gripper, Level.MIDDLE));
		chooser1.addOption("None",
				new InstantCommand());

		SmartDashboard.putData("Drop piece:", chooser1);

		chooser2.setDefaultOption("None", new InstantCommand());
		chooser2.addOption("Auto-balance (BOT MUST BE BEHIND CHARGING STATION)",
				SimpleAuto.autoBalance(swerve));
		chooser2.addOption("Exit community (BOT MUST NOT BE BEHIND CHARGING STATION)",
				SimpleAuto.exitCommunity(swerve));
		SmartDashboard.putData("Auto move action", chooser2);
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
	}

	@Override
	public void disabledInit() {
		SimpleAuto.lockIn(swerve).schedule(); // this should lock in the robot whenever it is disabled
	}

	@Override
	public void disabledPeriodic() {
	}

	@Override
	public void autonomousInit() {
		autonomousCommand = chooser1.getSelected().andThen(chooser2.getSelected());
		if (autonomousCommand != null)
			autonomousCommand.schedule();
		else
			System.err.print("fix your code");
	}

	@Override
	public void autonomousPeriodic() {
	}

	@Override
	public void teleopInit() {
		if (autonomousCommand != null)
			autonomousCommand.cancel();
	}

	@Override
	public void teleopPeriodic() {
	}

	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
	}

	@Override
	public void testPeriodic() {
	}

	@Override
	public void simulationInit() {
	}

	@Override
	public void simulationPeriodic() {
	}
}

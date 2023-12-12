package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.kSwerve;
import frc.robot.subsystems.Swerve;

public class AutoBalance extends CommandBase {
	private Swerve swerve;
	public PIDController pid = new PIDController(0.4, 0.3, .01);
	int x = 0;

	public AutoBalance(Swerve swerve) {
		SmartDashboard.putData("Auto-balance PID", pid);

		this.swerve = swerve;

		pid.setSetpoint(0);
		pid.setTolerance(3);
		NetworkTableInstance.getDefault();

		addRequirements(this.swerve);
	}

	public void execute() {
		double speed = pid.calculate(MathUtil.inputModulus(swerve.getPitch().getDegrees(), -180, 180));
		swerve.drive(new Translation2d(speed * 0.025, 0), 0, false, kSwerve.openLoop);
		if (pid.atSetpoint())
			x++;
		else
			x = 0;
	}

	@Override
	public boolean isFinished() {
		return x >= 10;
	}
}

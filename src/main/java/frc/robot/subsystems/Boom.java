package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import frc.robot.Constants.kBoom;
import frc.robot.Constants.kBoom.Level;

public class Boom extends SubsystemBase {
	TalonSRX controller = new TalonSRX(kBoom.controllerId);
	PIDController pid = new PIDController(0.5, 0, .02);
	Encoder encoder = new Encoder(2, 3);

	DoubleSolenoid pneumatic = new DoubleSolenoid(
			0, PneumaticsModuleType.CTREPCM,
			kBoom.forwardId, kBoom.reverseId);

	boolean raised = pneumatic.get() == Value.kForward;
	boolean rachetReleased = false;

	public Boom() {
		controller.setNeutralMode(NeutralMode.Brake);
		pid.setTolerance(0.5);
	}

	public CommandBase setTo(Level target) {
		CommandBase extend = runOnce(() -> pid.setSetpoint(-1))
				.andThen(new WaitCommand(0.1), runOnce(() -> rachetReleased = true)).unless(() -> rachetReleased)
				.andThen(runOnce(() -> pid.setSetpoint(target.getLength())), new WaitUntilCommand(pid::atSetpoint));

		CommandBase raise = runOnce(() -> pneumatic.set(target.isRaised() ? Value.kForward : Value.kReverse)).andThen(
				new WaitCommand(kBoom.raiseDelay).unless(() -> raised == target.isRaised()),
				runOnce(() -> raised = target.isRaised()));

		return target.isRaised() ? raise.andThen(extend) : extend.andThen(raise);
	}

	public double getEncoder() {
		return (double) encoder.get() / 2048.0;
	}

	public void zeroEncoder() {
		encoder.reset();
	}

	public void periodic() {
		double output = -pid.calculate(getEncoder());
		output = Math.abs(output) > 0.8 ? output / Math.abs(output) * 0.8 : output;
		controller.set(ControlMode.PercentOutput, output);

		SmartDashboard.putBoolean("Boom raised", raised);
		SmartDashboard.putBoolean("Boom within tolerance", pid.atSetpoint());
		SmartDashboard.putNumber("Boom Encoder", getEncoder());
		SmartDashboard.putNumber("Boom setpoint", pid.getSetpoint());
		SmartDashboard.putNumber("Boom output", output);
		SmartDashboard.putBoolean("Rachet released", rachetReleased);
	}
}

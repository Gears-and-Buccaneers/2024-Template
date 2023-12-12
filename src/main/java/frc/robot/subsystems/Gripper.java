package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class Gripper extends SubsystemBase {
	public final DoubleSolenoid gripper = new DoubleSolenoid(0,
			PneumaticsModuleType.CTREPCM,
			Constants.kGripper.forwardId, Constants.kGripper.reverseId);

	public void toggle() {
		if (gripper.get() == Value.kReverse)
			gripper.set(Value.kForward);
		else
			gripper.set(Value.kReverse);
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Gripper state",
				gripper.get() == Value.kReverse ? -1 : (gripper.get() == Value.kOff ? 0 : 1));
	}
}

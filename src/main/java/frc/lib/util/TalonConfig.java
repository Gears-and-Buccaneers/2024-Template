package frc.lib.util;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.pathplanner.lib.auto.PIDConstants;

public class TalonConfig extends TalonFXConfiguration {
	boolean invert;
	NeutralMode neutralMode;

	public TalonConfig(PIDConstants pid, double kF, boolean invert, NeutralMode neutralMode,
			SupplyCurrentLimitConfiguration lim) {
		super.slot0.kP = pid.kP;
		super.slot0.kI = pid.kI;
		super.slot0.kD = pid.kD;
		super.slot0.kF = kF;

		super.voltageCompSaturation = 12;
		super.supplyCurrLimit.currentLimit = 80;
		super.supplyCurrLimit.enable = true;

		super.supplyCurrLimit = lim;
		super.initializationStrategy = SensorInitializationStrategy.BootToZero;

		this.invert = invert;
		this.neutralMode = neutralMode;
	}

	public TalonConfig(PIDConstants pid, double kF, boolean invert, NeutralMode neutralMode,
			SupplyCurrentLimitConfiguration lim, double openLoopRamp, double closedloopRamp) {
		this(pid, kF, invert, neutralMode, lim);

		super.openloopRamp = openLoopRamp;
		super.closedloopRamp = closedloopRamp;
	}

	public TalonFX create(int deviceNumber) {
		TalonFX motor = new TalonFX(deviceNumber);

		motor.configFactoryDefault();
		motor.configAllSettings(this);

		motor.setInverted(invert);
		motor.setNeutralMode(neutralMode);
		motor.enableVoltageCompensation(true);

		motor.setSensorPhase(true);

		return motor;
	}
}

package frc.robot.subsystems;

//import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.math.CTREModuleState;
import frc.lib.math.Conversions;

import frc.robot.Constants.kSwerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

public class SwerveModule {
	public int moduleNumber;
	public TalonFX angleMotor;
	public TalonFX driveMotor;
	private CANCoder angleEncoder;
	public double lastAngle;
	private double angleOffset;

	// private final SimpleMotorFeedforward feedforward = new
	// SimpleMotorFeedforward(0.6716, 2.5913, 0.19321);

	public SwerveModule(int moduleNumber, int driveMotorID, int angleMotorID, int canCoderID, double angleOffset) {
		this.moduleNumber = moduleNumber;
		this.angleOffset = angleOffset;

		/* Angle Encoder Config */
		angleEncoder = new CANCoder(canCoderID);
		configAngleEncoder();

		angleMotor = kSwerve.angleConfig.create(angleMotorID);
		driveMotor = kSwerve.driveConfig.create(driveMotorID);

		lastAngle = 1000;
	}

	public void setToCurrent() {
		angleMotor.set(ControlMode.Position, angleMotor.getSelectedSensorPosition());
	}

	public double getAbsolutePosition() {
		return getEncoder() - angleOffset;
	}

	public double percentOutput;

	public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
		desiredState = CTREModuleState.optimize(desiredState, Rotation2d.fromDegrees(getAbsolutePosition()));

		if (isOpenLoop) {
			double percentOutput = desiredState.speedMetersPerSecond * kSwerve.maxSpeed;
			driveMotor.set(ControlMode.PercentOutput, percentOutput);
		} else {
			// double referenceVelocity = desiredState.speedMetersPerSecond;
			// double arbFeedForward =
			// feedforward.calculate(desiredState.speedMetersPerSecond) /
			// kSwerve.nominalVoltage;
			driveMotor.set(
					TalonFXControlMode.Velocity,
					desiredState.speedMetersPerSecond
							/ (Math.PI * kSwerve.wheelDiameter * kSwerve.driveGearRatio / 2048 * 10));
			// driveMotor.feed();
		}

		double angle = getIntegrated() + (desiredState.angle.getDegrees() - getAbsolutePosition());

		// Prevent rotating module if new angle is within 2 degrees of the last angle.
		// Prevents Jittering.
		angle = Math.abs(lastAngle - angle) <= 2 ? lastAngle : angle;

		angleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle,
				kSwerve.angleGearRatio));

		lastAngle = angle;
	}

	private void configAngleEncoder() {
		angleEncoder.configFactoryDefault();

		CANCoderConfiguration config = new CANCoderConfiguration();

		config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
		config.sensorDirection = kSwerve.canCoderInvert;
		config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
		config.sensorTimeBase = SensorTimeBase.PerSecond;

		angleEncoder.configAllSettings(config);
	}

	public double getEncoder() {
		return angleEncoder.getAbsolutePosition();
	}

	public double getIntegrated() {
		return Conversions.falconToDegrees(angleMotor.getSelectedSensorPosition(),
				kSwerve.angleGearRatio);
	}

	public double getVelocity() {
		return Conversions.falconToMPS(driveMotor.getSelectedSensorVelocity(),
				kSwerve.wheelCircumference, kSwerve.driveGearRatio);
	}

	public double getMeter() {
		return percentOutput;
	}

	public SwerveModulePosition getPos() {
		double distanceMeters = Conversions.falconToRPM(driveMotor.getSelectedSensorPosition(),
				kSwerve.driveGearRatio);

		Rotation2d angle = Rotation2d.fromDegrees(getAbsolutePosition());

		return new SwerveModulePosition(distanceMeters, angle);
	}
}

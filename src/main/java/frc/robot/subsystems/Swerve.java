package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kSwerve;

public class Swerve extends SubsystemBase {
	public SwerveDriveOdometry swerveOdometry;
	public PigeonIMU gyro;

	public Swerve() {
		gyro = new PigeonIMU(kSwerve.pigeonID);
		System.err.println(gyro.configFactoryDefault());

		offsetGyro(180);

		swerveOdometry = new SwerveDriveOdometry(kSwerve.swerveKinematics, getYaw(), getPos());
	}

	public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
		ChassisSpeeds speeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);

		if (fieldRelative)
			speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getYaw());
		setModuleStates(kSwerve.swerveKinematics.toSwerveModuleStates(speeds), isOpenLoop);
	}

	public void setModuleStates(SwerveModuleState[] desiredStates, boolean openLoop) {
		SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, kSwerve.maxSpeed);

		for (SwerveModule mod : kSwerve.mods)
			mod.setDesiredState(desiredStates[mod.moduleNumber], openLoop);
	}

	public void setModuleStates(SwerveModuleState[] desiredStates) {
		SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, kSwerve.maxSpeed);

		for (SwerveModule mod : kSwerve.mods)
			mod.setDesiredState(desiredStates[mod.moduleNumber], true);
	}

	public void setToCurrent() {
		for (SwerveModule mod : kSwerve.mods)
			mod.setToCurrent();
	}

	public Pose2d getPose() {
		return swerveOdometry.getPoseMeters();
	}

	public void resetOdometry(Pose2d pose) {
		swerveOdometry.resetPosition(getYaw(), getPos(), pose);
	}

	public SwerveModulePosition[] getPos() {
		SwerveModulePosition[] states = new SwerveModulePosition[4];
		for (SwerveModule mod : kSwerve.mods)
			states[mod.moduleNumber] = mod.getPos();
		return states;
	}

	public void zeroGyro() {
		gyro.setYaw(0);
	}

	public void offsetGyro(double angle) {
		gyro.setYaw(angle);
	}

	public Rotation2d getYaw() {
		return (kSwerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw())
				: Rotation2d.fromDegrees(gyro.getYaw());
	}

	public Rotation2d getPitch() {
		return (kSwerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getPitch())
				: Rotation2d.fromDegrees(gyro.getRoll());
	}

	public Rotation2d getRoll() {
		return (kSwerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getRoll())
				: Rotation2d.fromDegrees(gyro.getPitch());
	}

	@Override
	public void periodic() {
		swerveOdometry.update(getYaw(), getPos());

		SmartDashboard.putNumber("Pitch",
				Math.abs(MathUtil.inputModulus(getPitch().getDegrees(), -180, 180)));

		for (SwerveModule mod : kSwerve.mods) {
			SmartDashboard.putNumber("Mod" + mod.moduleNumber + " Cancoder", mod.getEncoder());
			SmartDashboard.putNumber("Mod" + mod.moduleNumber + " Integrated", mod.getIntegrated());
			SmartDashboard.putNumber("Mod" + mod.moduleNumber + " Velocity", mod.getVelocity());
			SmartDashboard.putNumber("Mod" + mod.moduleNumber + " meter", mod.getMeter());

			SmartDashboard.putNumber("Mod" + mod.moduleNumber + " Absolute Position", mod.getAbsolutePosition());
		}
	}
}

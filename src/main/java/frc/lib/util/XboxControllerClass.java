package frc.lib.util;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;

public final class XboxControllerClass {

	private static final double deadband = 0.05;

	public final Trigger A, B, X, Y, LB, RB, BACK, START, LS, RS, UP, UP_RIGHT, RIGHT, DOWN_RIGHT, DOWN, DOWN_LEFT,
			LEFT,
			UP_LEFT, CENTER;
	public final JoystickAxis LS_X, LS_Y, RS_X, RS_Y, LT, RT;

	public XboxControllerClass(int port) {
		XboxController joystick = new XboxController(port);

		LS_X = new JoystickAxis(joystick, Axis.kLeftX, deadband, 2);
		LS_Y = new JoystickAxis(joystick, Axis.kLeftY, deadband, 2);
		RS_X = new JoystickAxis(joystick, Axis.kRightX, deadband, 2);
		RS_Y = new JoystickAxis(joystick, Axis.kRightY, deadband, 2);

		LT = new JoystickAxis(joystick, Axis.kLeftTrigger, deadband, 3);
		RT = new JoystickAxis(joystick, Axis.kRightTrigger, deadband, 3);

		A = new JoystickButton(joystick, Button.kA.value);
		B = new JoystickButton(joystick, Button.kB.value);
		X = new JoystickButton(joystick, Button.kX.value);
		Y = new JoystickButton(joystick, Button.kY.value);
		LB = new JoystickButton(joystick, Button.kLeftBumper.value);
		RB = new JoystickButton(joystick, Button.kRightBumper.value);
		BACK = new JoystickButton(joystick, Button.kBack.value);
		START = new JoystickButton(joystick, Button.kStart.value);
		LS = new JoystickButton(joystick, Button.kLeftStick.value);
		RS = new JoystickButton(joystick, Button.kRightStick.value);

		UP = new Trigger(() -> joystick.getPOV() == 0);
		UP_RIGHT = new Trigger(() -> joystick.getPOV() == 45);
		RIGHT = new Trigger(() -> joystick.getPOV() == 90);
		DOWN_RIGHT = new Trigger(() -> joystick.getPOV() == 135);
		DOWN = new Trigger(() -> joystick.getPOV() == 180);
		DOWN_LEFT = new Trigger(() -> joystick.getPOV() == 225);
		LEFT = new Trigger(() -> joystick.getPOV() == 270);
		UP_LEFT = new Trigger(() -> joystick.getPOV() == 315);
		CENTER = new Trigger(() -> joystick.getPOV() == -1);
	}
}

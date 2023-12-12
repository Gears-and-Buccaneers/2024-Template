package frc.robot.subsystems;

import java.util.Arrays;

import org.xmlpull.v1.XmlPullParser;
import org.xmlpull.v1.XmlPullParserException;

import java.io.IOException;

import com.nosolojava.fsm.runtime.Context;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.nosolojava.fsm.impl.runtime.basic.BasicStateMachineEngine;
import com.nosolojava.fsm.impl.runtime.basic.BasicStateMachineFramework;
import com.nosolojava.fsm.runtime.StateMachineEngine;
import com.nosolojava.fsm.runtime.executable.CustomAction;
import com.nosolojava.fsm.parser.XppActionParser;

public class StateMachine extends SubsystemBase implements XppActionParser {
	Context ctx;

	public StateMachine() {
		BasicStateMachineFramework.DEBUG.set(true);

		try {
			StateMachineEngine engine = new BasicStateMachineEngine(Arrays.asList(this));
			engine.start();
			ctx = engine.startFSMSession(Constants.State.file.toURI());
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	@Override
	public String getNamespace() {
		return Constants.State.namespace;
	}

	@Override
	public CustomAction parseAction(XmlPullParser xpp) throws XmlPullParserException, IOException {
		switch (xpp.getName()) {
			case "enableDriving":
				Boolean drivingEnabled = xpp.getAttributeValue(null, "value").equals("1");
				return new CustomAction() {
					@Override
					public void run(Context context) {
						// robot.drivingEnabled = drivingEnabled;
						if (drivingEnabled) {
							System.out.println("...enabled driving");
						} else {
							System.out.println("...disabled driving");
						}
					}
				};

			case "robotToggle":
				String piece = xpp.getAttributeValue(null, "key");
				Boolean open = xpp.getAttributeValue(null, "value").equals("1");
				return new CustomAction() {
					@Override
					public void run(Context context) {
						// TODO: tell the robot to control the arm/gripper/boom
						System.out.println("Should " + (open ? "open/raise" : "close/lower") + " " + piece);
					}
				};

			case "doMagic":
				String magicKind = xpp.getAttributeValue(null, "magic");
				return new CustomAction() {
					@Override
					public void run(Context context) {
						// TODO: make the robot do the right kind of magic
						System.out.println("Time to start doing magic: " + magicKind);
					}
				};

			case "calibrate":
				return new CustomAction() {
					@Override
					public void run(Context context) {
						// TODO: tell the robot to calibrate
						System.out.println("Time to calibrate!");
					}
				};

			// This is for silly trace debugging, and probably encourages bad habits
			case "output":
				String message = xpp.getAttributeValue(null, "message");
				return new CustomAction() {
					@Override
					public void run(Context context) {
						System.out.println("\n" + message + "\n===================");
					}
				};

			default:
				return null;
		}
	}

}

package frc.lib.util;

import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants.kVision;

public class Camera {
	PhotonCamera c;
	Transform3d rel;

	public Camera(String name, Transform3d robotRelative) {
		c = new PhotonCamera(name);
		rel = robotRelative;
	}

	public static Map<Integer, Transform3d> tags() {
		Map<Integer, Reducer> data = new HashMap<>();

		for (Camera cam : kVision.cameras)
			for (PhotonTrackedTarget target : cam.c.getLatestResult().getTargets())
				if (target.getPoseAmbiguity() != -1 && target.getPoseAmbiguity() <= kVision.maxAmbiguity) {
					Reducer entry = data.get(target.getFiducialId());

					if (entry == null) {
						entry = new Reducer();
						data.put(target.getFiducialId(), entry);
					}

					entry.add(kVision.maxAmbiguity - target.getPoseAmbiguity(), target.getBestCameraToTarget().plus(cam.rel));
				}

		Map<Integer, Transform3d> res = new HashMap<>();

		for (Entry<Integer, Reducer> entry : data.entrySet())
			res.put(entry.getKey(), entry.getValue().average());

		return res;
	}

	public static class Reducer {
		double total = 0, x = 0, y = 0, z = 0;
		Translation3d translation = new Translation3d();

		Transform3d average() {
			return new Transform3d(translation.div(total), new Rotation3d(x / total, y / total, z / total));
		}

		void add(double confidence, Transform3d other) {
			translation = translation.plus(other.getTranslation().times(confidence));

			Rotation3d r = other.getRotation();
			x += r.getX() * confidence;
			y += r.getY() * confidence;
			z += r.getZ() * confidence;

			total += confidence;
		}
	}
}

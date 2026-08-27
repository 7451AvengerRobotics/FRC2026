package frc.robot.subsystems.Shooters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;

public class ShotCalc {
  double shotOffset = 0;
  Transform3d turretOffset = new Transform3d();
  InterpolatingDoubleTreeMap angleLerp = new InterpolatingDoubleTreeMap();
  InterpolatingDoubleTreeMap velocityLerp = new InterpolatingDoubleTreeMap();

  public ShotCalc(Transform3d turretOffset) {
    this.turretOffset = turretOffset;
    // angleLerp: distance (m) -> angle (rad)
    angleLerp.put(1.629 + shotOffset, 1.3072516100); // 15.1°
    angleLerp.put(2.067 + shotOffset, 1.3037609510); // 15.3°
    angleLerp.put(2.178 + shotOffset, 1.2955579040); // 15.77°
    angleLerp.put(2.200 + shotOffset, 1.2941616400); // 15.85°
    angleLerp.put(2.330 + shotOffset, 1.2835151320); // 16.46°
    angleLerp.put(2.390 + shotOffset, 1.2718214260); // 17.13°
    angleLerp.put(2.580 + shotOffset, 1.2603022530); // 17.79°
    angleLerp.put(2.640 + shotOffset, 1.2541936000); // 18.14°
    angleLerp.put(2.840 + shotOffset, 1.2250466020); // 19.81°
    angleLerp.put(2.860 + shotOffset, 1.2107349020); // 20.63°
    angleLerp.put(3.140 + shotOffset, 1.1475539830); // 24.25°
    angleLerp.put(3.420 + shotOffset, 1.1135200630); // 26.2°
    angleLerp.put(3.630 + shotOffset, 1.0960667700); // 27.2°
    angleLerp.put(3.500 + shotOffset, 1.0838494650); // 27.9°
    angleLerp.put(4.120 + shotOffset, 1.0314895880); // 30.9°
    angleLerp.put(4.240 + shotOffset, 1.0005972600); // 32.67°
    angleLerp.put(4.680 + shotOffset, 0.9684832019); // 34.51°
    angleLerp.put(4.970 + shotOffset, 0.9171705219); // 37.45°
    angleLerp.put(5.060 + shotOffset, 0.8749335540); // 39.87°
    angleLerp.put(5.190 + shotOffset, 0.8477064177); // 41.43°
    angleLerp.put(5.340 + shotOffset, 0.8382816397); // 41.97°
    angleLerp.put(5.510 + shotOffset, 0.8014551925); // 44.08°
    angleLerp.put(5.610 + shotOffset, 0.7686430026); // 45.96°
    angleLerp.put(5.670 + shotOffset, 0.7518878418); // 46.92°
    angleLerp.put(5.700 + shotOffset, 0.5846852994); // 56.5°

    // velocityLerp: angle (rad) -> velocity (m/s)
    velocityLerp.put(1.3072516100, 6.3764160460); // 15.1°
    velocityLerp.put(1.3037609510, 6.9474119980); // 15.3°
    velocityLerp.put(1.2955579040, 7.0199995490); // 15.77°
    velocityLerp.put(1.2941616400, 7.0359599800); // 15.85°
    velocityLerp.put(1.2835151320, 7.1072686120); // 16.46°
    velocityLerp.put(1.2718214260, 7.0846616950); // 17.13°
    velocityLerp.put(1.2603022530, 7.2141174950); // 17.79°
    velocityLerp.put(1.2541936000, 7.2347236220); // 18.14°
    velocityLerp.put(1.2250466020, 7.2439663810); // 19.81°
    velocityLerp.put(1.2107349020, 7.1756909430); // 20.63°
    velocityLerp.put(1.1475539830, 7.1229431880); // 24.25°
    velocityLerp.put(1.1135200630, 7.2300475180); // 26.2°
    velocityLerp.put(1.0960667700, 7.3390407720); // 27.2°
    velocityLerp.put(1.0838494650, 7.2021911380); // 27.9°
    velocityLerp.put(1.0314895880, 7.5326382080); // 30.9°
    velocityLerp.put(1.0005972600, 7.5620423180); // 32.67°
    velocityLerp.put(0.9684832019, 7.8102271870); // 34.51°
    velocityLerp.put(0.9171705219, 7.9621526530); // 37.45°
    velocityLerp.put(0.8749335540, 8.0311150000); // 39.87°
    velocityLerp.put(0.8477064177, 8.1340625400); // 41.43°
    velocityLerp.put(0.8382816397, 8.2305036130); // 41.97°
    velocityLerp.put(0.8014551925, 8.3914965200); // 44.08°
    velocityLerp.put(0.7686430026, 8.5306090010); // 45.96°
    velocityLerp.put(0.7518878418, 8.6155074580); // 46.92°
    velocityLerp.put(0.5846852994, 9.6663236710); // 56.5°
  }

  public double newGetPitch(double xf) {
    return Math.PI / 2 - angleLerp.get(xf);
  }

  public double mod(double angle) {
    return ((angle % (2 * Math.PI)) + (2 * Math.PI)) % (2 * Math.PI);
  }
}

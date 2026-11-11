package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShotCalc {
  double shotOffset = 0;
  public static InterpolatingDoubleTreeMap angleLerp = new InterpolatingDoubleTreeMap();

  public ShotCalc() {
    // angleLerp: distance (m) -> angle (rad)
    angleLerp.put(2.38 + shotOffset, 1.3028882870); // 74.65°
    angleLerp.put(2.52 + shotOffset, 1.2777555450); // 73.21°
    angleLerp.put(2.63 + shotOffset, 1.2575097260); // 72.05°
    angleLerp.put(2.72 + shotOffset, 1.2384856370); // 70.96°
    angleLerp.put(2.92 + shotOffset, 1.2264428650); // 70.27°
    angleLerp.put(3.07 + shotOffset, 1.1974704000); // 68.61°
    angleLerp.put(3.15 + shotOffset, 1.1705923290); // 67.07°
    angleLerp.put(3.39 + shotOffset, 1.1506955760); // 65.93°
    angleLerp.put(3.64 + shotOffset, 1.1218976430); // 64.28°
    angleLerp.put(3.82 + shotOffset, 1.0993828960); // 62.99°
    angleLerp.put(4.25 + shotOffset, 1.0796606750); // 61.86°
    angleLerp.put(4.41 + shotOffset, 1.0564477960); // 60.53°
    angleLerp.put(4.72 + shotOffset, 1.0088003080); // 57.80°
    angleLerp.put(4.99 + shotOffset, 0.9848892969); // 56.43°
    angleLerp.put(5.17 + shotOffset, 0.9660397410); // 55.35°
    angleLerp.put(5.29 + shotOffset, 0.9180431865); // 52.60°
    angleLerp.put(5.41 + shotOffset, 0.8677777041); // 49.72°
    angleLerp.put(5.58 + shotOffset, 0.8513716091); // 48.78°
    angleLerp.put(5.66 + shotOffset, 0.8307767239); // 47.60°
    angleLerp.put(5.79 + shotOffset, 0.8011061267); // 45.90°
  }

  public static double newGetPitch(double xf) {
    return Math.PI / 2 - angleLerp.get(xf);
  }

  public double mod(double angle) {
    return ((angle % (2 * Math.PI)) + (2 * Math.PI)) % (2 * Math.PI);
  }
}
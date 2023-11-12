package frc.robot.helpers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

//dont worry about it :)
public class Vector2 {
  public double x;
  public double y;

  public Vector2(double x, double y) {
    this.x = x;
    this.y = y;
  }

  public Vector2() {
    this.x = 0;
    this.y = 0;
  }

  public Vector2(Pose2d pose) {
    this.x = pose.getX();
    this.y = pose.getY();
  }

  public double getAngle() {
    if (x == 0 && y == 0) {
      return 0;
    }
    if (x < 0) {
      return Math.atan(y / x) + Math.PI;
    } else {
      return (Math.atan(y / x) + 2 * Math.PI) % (2 * Math.PI);
    }
  }

  public double getMagnitude() {
    return Math.sqrt(x * x + y * y);
  }

  public void multiply(double v) {
    x *= v;
    y *= v;
  }

  public static Vector2 fromPolar(double rads, double mag) {
    return new Vector2(Math.cos(rads) * mag, Math.sin(rads) * mag);
  }

  public String toString() {
    return "<" + x + ", " + y + ">";
  }

  public String toString(int decimals) {
    double pow = Math.pow(10, decimals);
    return "<" + Math.round(x * pow) / pow + ", " + Math.round(y * pow) / pow + ">";
  }

  public String toStringPolar(int decimals) {
    double pow = Math.pow(10, decimals);
    return "<" + Math.round(getMagnitude() * pow) / pow + ", " + Math.round(getAngle() * 180 / Math.PI * pow) / pow
        + "°>";
  }

  public Translation2d toTranslation() {
    return new Translation2d(x, y);
  }
}
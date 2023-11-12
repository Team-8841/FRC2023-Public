package frc.robot.helpers;

public class SwerveTiltOutput {
    public double tilt;
    public Vector2 tiltDirection;
    public SwerveTiltOutput(double tilt, Vector2 tiltDirection) {
        this.tilt = tilt;
        this.tiltDirection = tiltDirection;
    }
}

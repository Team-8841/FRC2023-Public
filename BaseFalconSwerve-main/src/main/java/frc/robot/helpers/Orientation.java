package frc.robot.helpers;

public class Orientation {
    public double yaw;
    public double pitch;
    public double roll;
    public Vector2 position;
    public Orientation(double yaw, double pitch, double roll, Vector2 position) {
        this.yaw = yaw;
        this.pitch = pitch;
        this.roll = roll;
        this.position = position;
    }
}

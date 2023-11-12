package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;

public class AutoBalance extends CommandBase {

    private Swerve s_Swerve;
    final private double TOLERANCE_VALUE = 13.5;
    private double forSpeed, revSpeed;
    boolean robotBalanced; // the tolerance val is 2 degrees

    public AutoBalance(Swerve swerve, double fspeed, double rspeed) {
        s_Swerve = swerve;
        forSpeed = fspeed;
        revSpeed = rspeed;

        addRequirements(swerve);
    }

    public void balanceSwerve() {
        SmartDashboard.putBoolean("Is robot balanced", robotBalanced);
        SmartDashboard.putNumber("Roll Val", s_Swerve.gyro.getPitch());

        if (s_Swerve.gyro.getPitch() > TOLERANCE_VALUE) {
            robotBalanced = false;
            DriverStation.reportWarning("forward balance", false);
            s_Swerve.drive(
                    new Translation2d(forSpeed, 0).times(Constants.Swerve.maxSpeed),
                    0 * Constants.Swerve.maxAngularVelocity,
                    false,
                    true);
            // GO FORWARDS
        } else if (s_Swerve.gyro.getPitch() < -(TOLERANCE_VALUE)) {
            DriverStation.reportWarning("rev balance", false);
            robotBalanced = false;
            s_Swerve.drive(
                    new Translation2d(-revSpeed, 0).times(Constants.Swerve.maxSpeed),
                    0 * Constants.Swerve.maxAngularVelocity,
                    false,
                    true);
        } else {
            robotBalanced = true;
            s_Swerve.drive(
                    new Translation2d(0, 0).times(Constants.Swerve.maxSpeed),
                    0 * Constants.Swerve.maxAngularVelocity,
                    false,
                    true);
            // isFinished();
        }

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        balanceSwerve();

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}

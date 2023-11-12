package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoSwerve extends CommandBase {
    private Swerve s_Swerve;
    private double translationSup;

    public AutoSwerve(Swerve s_Swerve, double translationSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
    }

    @Override
    public void execute() {
        /* Drive */
        s_Swerve.drive(
                new Translation2d(translationSup, 0).times(Constants.Swerve.autoMaxSpeed),
                0 * Constants.Swerve.maxAngularVelocity,
                true,
                true);
    }
}
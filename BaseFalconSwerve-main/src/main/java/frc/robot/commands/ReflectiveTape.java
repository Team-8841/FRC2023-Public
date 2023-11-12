package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ReflectiveTape extends CommandBase {
    private Swerve s_Swerve;
    private double translationSup;
    Joystick controller;

    public ReflectiveTape(Swerve s_Swerve, Joystick controller) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
        this.controller = controller;
    }

    @Override
    public void execute() {
        /* Drive */
        s_Swerve.drive(
                new Translation2d(0, 0).times(Constants.Swerve.autoMaxSpeed),
                0 * Constants.Swerve.maxAngularVelocity,
                false,
                true);
    }
}
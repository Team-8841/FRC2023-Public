package frc.robot;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DSConstants;
import frc.robot.commands.*;
import frc.robot.helpers.Vector2;
import frc.robot.subsystems.*;

public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(DSConstants.gamepadPort);
    private final Joystick copilotDS = new Joystick(DSConstants.copilotDSPort);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kStart.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton grabberOut = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton grabberSecond = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton armExtend = new JoystickButton(copilotDS, DSConstants.armExtension);
    private final JoystickButton grabberIn = new JoystickButton(copilotDS, DSConstants.gripperOn);

    /* Subsystems */
    public final static Swerve s_Swerve = new Swerve();
    private final Arm s_Arm = new Arm();
    private final CargoHandler s_CargoHandler = new CargoHandler();
    private final Vision s_Vision = new Vision();

    private final Compressor p_compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);

    private static SwerveAutoBuilder swerveAutoBuilder;

    SendableChooser<List<PathPlannerTrajectory>> autoChooser = new SendableChooser<>();

    public double timeArmKnobChanged = Timer.getMatchTime();

    private void setUpEventMap() {

        /* Setup pathplanner event tags to their commands here */

        AutoConstants.eventMap.clear();
        AutoConstants.eventMap.put("balance", new AutoBalance(s_Swerve, 0.22, 0.22));
        AutoConstants.eventMap.put("revBalance", new AutoBalance(s_Swerve, 0.3, 0.3));
        AutoConstants.eventMap.put("scoreCube", new CubeScore(s_Arm));
        AutoConstants.eventMap.put("scoreCone", new ConeScore(s_Arm));
        AutoConstants.eventMap.put("intakeIn", new IntakeInAndOut(false, true, false, s_CargoHandler));
        AutoConstants.eventMap.put("sensorIntakeIn", new IntakeInAndOut(true, false, false, s_CargoHandler));
        AutoConstants.eventMap.put("intakeOut", new IntakeInAndOut(false, false, true, s_CargoHandler));
        AutoConstants.eventMap.put("intakeOff", new IntakeInAndOut(false, false, false, s_CargoHandler));
    }

    private void setUpAutos() {
        /* Path planner paths */
        setUpEventMap();

        autoChooser.setDefaultOption("Center Balance",
                PathPlanner.loadPathGroup("center_balance", new PathConstraints(2.8, 1)));
        autoChooser.addOption("Center Cone Balance",
                PathPlanner.loadPathGroup("center_cone_balance", new PathConstraints(2.5, 1)));
        autoChooser.addOption("Center Cone Rev Balance",
                PathPlanner.loadPathGroup("center_cone_rev_balance", new PathConstraints(2.5, 1)));
        autoChooser.addOption("Smooth eateverything",
                PathPlanner.loadPathGroup("smooth_eateverything", new PathConstraints(4.5, 4)));
        autoChooser.addOption("bump foward", PathPlanner.loadPathGroup("bump_forward", new PathConstraints(3,1)));

        SmartDashboard.putData(autoChooser);

    }

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // use this only for the pit and tuning
        //PathPlannerServer.startServer(5811);

        /* Setup auto stuff */
        setUpAutos();
        SmartDashboard.putData(autoChooser);

        s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                        s_Swerve,
                        () -> {
                            Vector2 speed = new Vector2(-driver.getRawAxis(translationAxis),
                                    -driver.getRawAxis(strafeAxis));
                            speed.multiply((Constants.Swerve.joystickCurveConstant
                                    * Math.pow(speed.getMagnitude(), Constants.Swerve.joystickCurveConstant - 1)
                                    + 1 / Math.sqrt(speed.getMagnitude()))
                                    / (Constants.Swerve.joystickCurveConstant + 1));

                            return speed.x;
                        },
                        () -> {
                            Vector2 speed = new Vector2(-driver.getRawAxis(translationAxis),
                                    -driver.getRawAxis(strafeAxis));
                            speed.multiply((Constants.Swerve.joystickCurveConstant
                                    * Math.pow(speed.getMagnitude(), Constants.Swerve.joystickCurveConstant - 1)
                                    + 1 / Math.sqrt(speed.getMagnitude()))
                                    / (Constants.Swerve.joystickCurveConstant + 1));

                            return speed.y;
                        },
                        () -> {
                            double v = driver.getRawAxis(rotationAxis);
                            return -Math.signum(v) * (Constants.Swerve.rotationCurveConstant
                                    * Math.pow(Math.abs(v), Constants.Swerve.rotationCurveConstant)
                                    + Math.sqrt(Math.abs(v))) / (Constants.Swerve.rotationCurveConstant + 1);
                        },
                        () -> robotCentric.getAsBoolean(),
                        new CommandXboxController(DSConstants.gamepadPort)));

        // Configure the button bindings
        configureButtonBindings();

        // Compressor things
        p_compressor.enableDigital();
        p_compressor.disable();

        /* Default commands */

        s_CargoHandler.setDefaultCommand(new RunCommand(() -> {
            s_CargoHandler.intakeInAndOut(copilotDS.getRawButton(DSConstants.quickFeed),
                    copilotDS.getRawButton(DSConstants.fullFeed),
                    copilotDS.getRawButton(DSConstants.intakeOUT));

            s_CargoHandler.TTRightLeft(copilotDS.getRawButton(DSConstants.turnTableClockwise),
                    copilotDS.getRawButton(DSConstants.turnTableCounterClockwise));
        }, s_CargoHandler));

        s_Arm.setArmSetPoint(ArmConstants.home);
        s_Arm.setDefaultCommand(new RunCommand(() -> {
            if (copilotDS.getRawButton(DSConstants.manualArmMode)) {
                s_Arm.SensorControl(getArmSpeed(), 0, true);
            } else {
                //s_Arm.UpdatePID();
                s_Arm.SensorControl(0, s_Arm.getArmSetPoint(), false); 
            }

            s_Arm.grabSensorControl(armExtend.getAsBoolean(), grabberIn.getAsBoolean(), false);
        }, s_Arm));

        s_Vision.setDefaultCommand(new RunCommand(() -> {
            s_Vision.updateStatus();
        }, s_Vision));
    }

    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

        new JoystickButton(copilotDS, DSConstants.compressor).onTrue(new InstantCommand(() -> {
            p_compressor.disable();
        }));

        new JoystickButton(copilotDS, DSConstants.compressor).onFalse(new InstantCommand(() -> {
            p_compressor.enableDigital();
        }));

        new JoystickButton(copilotDS, DSConstants.armPivot).onTrue(new InstantCommand(() -> {
            s_Arm.setArmPivot(!s_Arm.getArmPivot());
        }));

        new JoystickButton(copilotDS, DSConstants.armPosHome).onTrue(new InstantCommand(() -> {
            s_Arm.setArmSetPoint(ArmConstants.home);
            DriverStation.reportWarning("Arm setpoint set to home", true);
        }));

        new JoystickButton(copilotDS, DSConstants.armPosMid).onTrue(new InstantCommand(() -> {
            s_Arm.setArmSetPoint(ArmConstants.middle);
            DriverStation.reportWarning("Arm setpoint set to middle", true);
        }));

        new JoystickButton(copilotDS, DSConstants.armPosMidDrop).onTrue(new InstantCommand(() -> {
            s_Arm.setArmSetPoint(ArmConstants.middleDrop);
            DriverStation.reportWarning("Arm setpoint set to middle drop", true);
        }));

        new JoystickButton(copilotDS, DSConstants.armPosTopDrop).onTrue(new InstantCommand(() -> {
            s_Arm.setArmSetPoint(ArmConstants.topConeDrop);
            DriverStation.reportWarning("Arm setpoint set to top cone drop", true);
        }));

        new JoystickButton(copilotDS, DSConstants.armPosTop).onTrue(new InstantCommand(() -> {
            s_Arm.setArmSetPoint(ArmConstants.top);
            DriverStation.reportWarning("Arm setpoint set to top", true);
        }));

        new JoystickButton(copilotDS, DSConstants.armPivot).onTrue(new InstantCommand(() -> {
            s_Arm.setArmPivot(!s_Arm.getArmPivot());
        }));

        grabberOut.whileTrue(new RunCommand(() -> {
            //if(s_Arm.getArmSetPoint() == ArmConstants.top && s_Arm.getArmPivot() && grabberSecond.getAsBoolean()) {
            if(grabberSecond.getAsBoolean()){
                DriverStation.reportWarning("Double rainbow across the sky", false);
                s_Arm.setArmPivot(false);
                s_Arm.grabSensorControl(false, false, true);
            } else {
                s_Arm.setGripperSpeed(ArmConstants.gripperMotorOutSpeed);
            }
        }));

        grabberOut.onFalse(new InstantCommand(() -> {
            s_Arm.setGripperSpeed(0);
        }));

    }

    private double getArmSpeed() {
        if (copilotDS.getRawAxis(DSConstants.arm) < 0.03) {
            return -ArmConstants.manualDownSpeed;
        } else if (copilotDS.getRawAxis(DSConstants.arm) > .1) {
            return ArmConstants.manualUpSpeed;
        } else {
            return 0.0;
        }
    }

    private static Command buildAuto(List<PathPlannerTrajectory> trajs, Boolean firstPath) {
        swerveAutoBuilder = new SwerveAutoBuilder(
                s_Swerve::getPose,
                s_Swerve::resetOdometry,
                frc.robot.Constants.Swerve.swerveKinematics,
                new PIDConstants(Constants.AutoConstants.kPXController, 0, 0),
                new PIDConstants(Constants.AutoConstants.kPThetaController, 0, 0),
                s_Swerve::setModuleStates,
                Constants.AutoConstants.eventMap,
                true,
                s_Swerve);

        if (firstPath) {
            s_Swerve.zeroGyro();
        }

        return swerveAutoBuilder.fullAuto(trajs);

    }

    public Command getAutonomousCommand() {
        return buildAuto(autoChooser.getSelected(), true);
    }
}

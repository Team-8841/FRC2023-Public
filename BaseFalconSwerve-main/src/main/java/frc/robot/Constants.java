package frc.robot;

import java.util.HashMap;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.05;

    // Swerve Drive Constants
    public static final class Swerve {
        public static final int pigeonID = 1;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        public static final COTSFalconSwerveConstants chosenModule = // TODO: This must be tuned to specific robot
                COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(25); // TODO: This must be tuned to specific robot
        public static final double wheelBase = Units.inchesToMeters(25); // TODO: This must be tuned to specific robot
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        public static final double joystickCurveConstant = 3.96;
        public static final double rotationCurveConstant = 3.96;

        public static double autoRotationCorrectionPID_P = 0.5;
        public static double autoRotationCorrectionPID_I = 0;
        public static double autoRotationCorrectionPID_D = 0;

        /*
         * Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional
         * rectangular/square 4 module swerve
         */
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
        // public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;
        public static final boolean driveMotorInvert = true;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = chosenModule.canCoderInvert;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 40;
        public static final int drivePeakCurrentLimit = 65;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /*
         * These values are used by the drive falcon to ramp in open loop and closed
         * loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
         */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKF = chosenModule.angleKF;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.05; // TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /*
         * Drive Motor Characterization Values
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE
         */
        public static final double driveKS = (0.32 / 12); // TODO: This must be tuned to specific robot
        public static final double driveKV = (1.51 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5; // TODO: This must be tuned to specific robot
        public static final double slowModeSpeed = 2.5;
        public static final double autoMaxSpeed = 1.5; // TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; // TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /*
         * Module Specific Constants *
         * /* Front Left Module - Module 0
         */
        public static final class Mod0 { // TODO: This must be tuned to specific robot
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 1;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(330.64 + 180);// 150);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { // TODO: This must be tuned to specific robot
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 6;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(218.144 + 180); // 41.39);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 { // TODO: This must be tuned to specific robot
            public static final int driveMotorID = 8;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 9;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(278.964 + 180); // 98.43);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { // TODO: This must be tuned to specific robot
            public static final int driveMotorID = 11;
            public static final int angleMotorID = 10;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(195.38 + 180);// 15.46);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants { // TODO: The below constants are used in the example auto, and must be
                                              // tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

        public static HashMap<String, Command> eventMap = new HashMap<>();

        public static double simpleAutoTranslationSpeed = 0.35;
        public static double simpleAutoTranslationSpeedOnBalance = 0.2;
    }

    /* Constants used in the RobotContainer.java */
    public static final class DSConstants {

        public static final int gamepadPort = 0; // XBox controller port
        public static final int copilotDSPort = 1; // copilot board port

        // Driver Station Knobs (Analog Inputs)
        public static final int arm = 0; // Manual Arm Control Joystick Y-axis

        // Driver Station Buttons (Digital Inputs)
        public static final int compressor = 8; // Compressor Override
        public static final int manualArmMode = 7; // Manual Arm Mode Enable
        public static final int intakeOUT = 1; // Intake OUT Momentary Button
        public static final int armPivot = 16; // Center red button
        public static final int armExtension = 15; // Right red button
        public static final int quickFeed = 4; // Top Row Switch Position
        public static final int fullFeed = 5; // Bottom Row Switch Position
        public static final int turnTableClockwise = 2; // Turn Table on Switch Position - its the backwards one
        public static final int turnTableCounterClockwise = 3; // Turn Table off Switch Position - its the forward one
        public static final int gripperOn = 6; // Gripper on Switch Position
        public static final int armPosTop = 9;
        public static final int armPosTopDrop = 10;
        public static final int armPosMid = 11;
        public static final int armPosMidDrop = 12;
        public static final int armPosHome = 14;

        // I dont think we need this stuff the gamepad is refrenced in a new way this
        // year.
        // Gamepad Buttons (Digital Inputs)
        public static final int buttonA = 1; // NOT USED
        public static final int buttonB = 2; // NOT USED
        public static final int buttonX = 3; // NOT USED
        public static final int buttonY = 4; // Limelight Toggle Button
        public static final int buttonLB = 5; // Gripper Toggle Button
        public static final int buttonRB = 6; // Score Button
        public static final int buttonSelect = 7; // NOT USED
        public static final int buttonStart = 8; // Reset Sensors

        // Gamepad (Analog Inputs)
        public static final int axisLX = 0;
        public static final int axisLY = 1;
        public static final int triggerL = 2; // NOT USED
        public static final int triggerR = 3; // NOT USED
        public static final int axisRX = 4;
        public static final int axisRY = 5;

        /* Arm Knob values */
        public static final double posHome = 0.118110;
        public static final double posMidDrop = 0.094488;
        public static final double posMid = 0.062992;
        public static final double posTopDrop = 0.031496;
        public static final double posTop = 0.000000;
        public static final double[] armKnobvalues = { posHome, posMidDrop, posMid, posTopDrop, posTop }; // home, floor, middle, top

    }

    /* Constatns used in the arm subsystem */
    public static final class ArmConstants {

        /* Motor IDs */
        public static final int leadArmMotorID = 16; // Right motor
        public static final int followArmMotorID = 17; // Left motor
        public static final int gripperMotorID = 18;

        /* Solenoid IDs */
        public static final int armPivotSolenoid = 0;
        public static final int armExtenstionSolenoid = 1;

        /* Sensor IDs */
        public static final int grabberSensorPort = 1;
        public static final int armBottomLimit = 2;
        public static final int armEncoderPortA = 3;
        public static final int armEncoderPortB = 4;

        /* Motor settings */
        public static final int currentLimit = 60;

        /* Motor speeds set as a % between -1 and 1 */
        public static final double gripperMotorInSpeed = 0.5;
        public static final double gripperMotorOutSpeed = -0.3;
        public static final double manualUpSpeed = 0.3;
        public static final double manualDownSpeed = 0.15;

        /* PID Constants */
        public static final double kP = 0.002000; //0.002100
        public static final double kI = 0.000090; // 0.000085
        public static final double kD = 0.000100;
        public static final double kFF = 0.0;

        public static final double pidUpSpeedMultiply = 1;

        /* Arm Setpoints */
        public static final double home = 0.0;
        public static final double middleDrop =380;// 332;
        public static final double middle = 455; //old 450
        public static final double topConeDrop = 635; //old 620
        public static final double top = 750; // old 710
        public static final double pivotSetPoint = 300;
        public static final double[] armSetpoints = { home, middleDrop, middle, topConeDrop, top };// home, floor, middle, top
    }

    /* Constants used in the cargo handler subsystem */
    public static final class CHConstants {

        /* Motor IDs */
        public static final int horizontalMotorID = 13;
        public static final int verticalMotorID = 14;
        public static final int gripperMotorID = 15;
        public static final int turnTableMotorID = 19;

        /* Solenoid IDs */

        /* Sensor IDs */
        public static final int rightBowlSensor = 8;
        public static final int leftBowlSensor = 5;
        public static final int intakeSensor = 9;

        /* Motor settings */
        public static final int currentLimit = 80;

        /* Motor speeds set as a % between -1 and 1 */
        public static final double verticalIntakeSpeed = 1;
        public static final double horizontalIntakeSpeed = 1;
        public static final double turnTableSpeed = .2;

        /* Vertical intake PID values */
        public static final double kP = 0.5;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kFF = 0;
        public static final double kMinOutput = 0;
        public static final double kMaxOutput = 1;

        /* Other values */

    }

    /* Constants used in the vision subsystem */
    public static final class VisionConstants {

        /* Camera settings */

        /* Measurements for distance function */
        public static final double heightOfCamera = 43; // Height of camera from floor
        public static final double heightOfTarget = 29; // Height of our target
        public static final double angleOfCamera = -20; // Angle the camera is mounted at
    }

    /* Constatnts used in the lighting subsystem */
    public static final class LightingConstants {
        public static final int dataLine = 7;
    }
}

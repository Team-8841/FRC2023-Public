package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DSConstants;

public class Arm extends SubsystemBase {

    /* I2C Port for color sensor */
    // private final I2C.Port colorSensorPort = I2C.Port.kOnboard;

    /* Motors */
    private final CANSparkMax m_leadMotor = new CANSparkMax(ArmConstants.leadArmMotorID, MotorType.kBrushed);
    private final CANSparkMax m_followMotor = new CANSparkMax(ArmConstants.followArmMotorID, MotorType.kBrushed);
    private final CANSparkMax m_gripperMotor = new CANSparkMax(ArmConstants.gripperMotorID, MotorType.kBrushed);

    /* Sensors */
    private final Encoder s_armEncoder = new Encoder(ArmConstants.armEncoderPortA, ArmConstants.armEncoderPortB);
    private final DigitalInput s_armBottomLimit = new DigitalInput(ArmConstants.armBottomLimit);
    private final DigitalInput s_grabberSensor = new DigitalInput(ArmConstants.grabberSensorPort);

    /* Solenoids */
    private final Solenoid p_armPivotSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM,
            ArmConstants.armPivotSolenoid);
    private final Solenoid p_armExtenstionSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM,
            ArmConstants.armExtenstionSolenoid);

    PIDController anglePID = new PIDController(ArmConstants.kP, ArmConstants.kI,
            ArmConstants.kD);

    private double pidUpSpeedMultiply = ArmConstants.pidUpSpeedMultiply;
    private double armSetPoint = ArmConstants.home;
    
     /*public void UpdatePID() {
        double angleP = SmartDashboard.getNumber("Arm angle PID P",
        ArmConstants.kP);
        double angleI = SmartDashboard.getNumber("Arm angle PID I",
        ArmConstants.kI);
        double angleD = SmartDashboard.getNumber("Arm angle PID D",
        ArmConstants.kD);
        
        pidUpSpeedMultiply = SmartDashboard.getNumber("Arm Up Speed Multiply",
        pidUpSpeedMultiply);
        if (angleP != anglePID.getP() || angleI != anglePID.getI() || angleD != anglePID.getD()) {
            anglePID.setP(angleP);
            anglePID.setI(angleI);
            anglePID.setD(angleD);
        }
     
     }*/
    

    public Arm() {

        /* Configure motors */
        configureSparkMax(m_leadMotor, IdleMode.kBrake);
        configureSparkMax(m_followMotor, IdleMode.kBrake);
        configureSparkMax(m_gripperMotor, IdleMode.kBrake);

        
         /*SmartDashboard.putNumber("Arm angle PID P",
         ArmConstants.kP);
         SmartDashboard.putNumber("Arm angle PID I",
         ArmConstants.kI);
         SmartDashboard.putNumber("Arm angle PID D",
         ArmConstants.kD);
         SmartDashboard.putNumber("Arm Up Speed Multiply",
         pidUpSpeedMultiply);*/
        

        /* Motor settings */
        m_leadMotor.setInverted(false);

        m_followMotor.follow(m_leadMotor, true);

    }

    @Override
    public void periodic() {
        updateStatus();
        //UpdatePID();
        if (m_leadMotor.get() < 0 && s_armBottomLimit.get()) {
            m_leadMotor.set(0);
        }

        // Very important do not delete keeps arm from frying
        if (getArmBottomLimit()) {
            s_armEncoder.reset();
        }
    }
    
    private void configureSparkMax(CANSparkMax spark, IdleMode idleMode) {
        spark.restoreFactoryDefaults();
        spark.setSmartCurrentLimit(ArmConstants.currentLimit);
        spark.setIdleMode(idleMode);
    }

    /* Getter/Setter methods */

    public void setArmSetPoint(double angle) {
        armSetPoint = angle;
    }

    public double getArmSetPoint() {
        return armSetPoint;
    }

    /* Motors */

    public void setArmAngle(double angle) {
        armSetPoint = angle;
        double val = anglePID.calculate(getArmAngle() - angle);
        if (val > 0) {
            val *= pidUpSpeedMultiply;
        }
        SmartDashboard.putNumber("arm pid output", val);
        SmartDashboard.putNumber("angle diff", (getArmAngle() - angle));
        if(getArmBottomLimit() && val < 0)
        {
            setArmSpeed(0);
        } else {
            setArmSpeed(val);
        }
    }

    public void setArmSpeed(double speed) {
        // SmartDashboard.putNumber("speed", speed);
        if (speed < 0 && s_armBottomLimit.get()) {
            m_leadMotor.set(0);
        }
        m_leadMotor.set(speed);
    }

    public void setGripperSpeed(double speed) {

        if(getGrabberSensor() && speed > 0) {
            m_gripperMotor.set(0);
        } else {
            m_gripperMotor.set(speed);
        }
    }

    /* Solenoids */

    public void setArmPivot(boolean state) {
        p_armPivotSolenoid.set(state);
    }

    public boolean getArmPivot() {
        return p_armPivotSolenoid.get();
    }

    public void setArmExtension(boolean state) {
        p_armExtenstionSolenoid.set(!state);
    }

    public boolean getArmExtension() {
        return p_armExtenstionSolenoid.get();
    }

    /* Sensors */

    public double getArmAngle() {
        return s_armEncoder.get();
    }

    public boolean getArmBottomLimit() {
        return !s_armBottomLimit.get();
    }

    public boolean getGrabberSensor() {
        return !s_grabberSensor.get();
    }

    public void resetEncoder() {
        s_armEncoder.reset();
    }

    public void grabSensorControl(boolean extend, boolean in, boolean out) {
        setArmExtension(extend);
        if(in) {
            if(getGrabberSensor()) {
                setGripperSpeed(0);
            } else {
                setGripperSpeed(ArmConstants.gripperMotorInSpeed);
            }
        } else if (out) {
            setGripperSpeed(ArmConstants.gripperMotorOutSpeed);
        } else {
            setGripperSpeed(0);
        }
    }

    public void SensorControl(double joystick, double targetPos, boolean manual) { // TODO
        if (manual) {

            if (getArmBottomLimit() && joystick > 0) {
                setArmSpeed(joystick);
            } else if (getArmBottomLimit() && joystick < 0) {
                setArmSpeed(0);
            } else {
                setArmSpeed(joystick);
            }

            if (getArmBottomLimit()) {
                resetEncoder();

            }

        } else {
            setArmSetPoint(targetPos);
            
            setArmAngle(armSetPoint);            
        }
    }

    private void updateStatus() {
        SmartDashboard.putNumber("Arm angle", getArmAngle());
        SmartDashboard.putBoolean("Arm Bottom Limit", getArmBottomLimit());
        SmartDashboard.putBoolean("Grabber Sensor", getGrabberSensor());
        SmartDashboard.putNumber("Arm Set Point", getArmSetPoint());
    }

}

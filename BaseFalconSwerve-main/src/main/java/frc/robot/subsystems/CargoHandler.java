package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CHConstants;

public class CargoHandler extends SubsystemBase {

    /* Motors */
    private final CANSparkMax m_hIntake = new CANSparkMax(CHConstants.horizontalMotorID,
            MotorType.kBrushed);
    private final CANSparkMax m_vIntake = new CANSparkMax(CHConstants.verticalMotorID,
            MotorType.kBrushed);
    private final CANSparkMax m_turnTable = new CANSparkMax(CHConstants.turnTableMotorID, MotorType.kBrushed);

    /* Sensors */
    private final DigitalInput s_intakeSensor = new DigitalInput(CHConstants.intakeSensor);
    private final DigitalInput s_leftBowlSensor = new DigitalInput(CHConstants.leftBowlSensor);
    private final DigitalInput s_rightBowlSensor = new DigitalInput(CHConstants.rightBowlSensor);

    /* Lights */
    private final static Solenoid m_leds = new Solenoid(PneumaticsModuleType.CTREPCM, 7);
    

    public CargoHandler() {

        /* Configure motors */
        configureSparkMax(m_hIntake, IdleMode.kBrake);
        configureSparkMax(m_vIntake, IdleMode.kBrake);
        configureSparkMax(m_turnTable, IdleMode.kBrake);

        m_hIntake.setInverted(false);
        m_vIntake.setInverted(true);

    }

    @Override
    public void periodic() {
        updateStatus();
    }

    private void configureSparkMax(CANSparkMax spark, IdleMode idleMode) {
        spark.restoreFactoryDefaults();
        spark.setSmartCurrentLimit(CHConstants.currentLimit);
        spark.setIdleMode(idleMode);
    }

    /* Getter/Setter methods */

    private void setVIntakeSpeed(double speed) {
        m_vIntake.set(speed);
    }

    private void setHIntakeSpeed(double speed) {
        m_hIntake.set(speed);
    }

    public void setTTableSpeed(double speed) {
        m_turnTable.set(speed);
    }

    public static void setLeds(boolean state) {
        m_leds.set(state);
    }

    public boolean getIntakeSensor() {
        return !s_intakeSensor.get();
    }

    public boolean getLeftBowlSensor() {
        return !s_leftBowlSensor.get();
    }

    public boolean getRightBowlSensor() {
        return !s_rightBowlSensor.get();
    }

    /* Intake Control */

    public void intakeInAndOut(boolean sensoredIn, boolean intakeIn, boolean intakeOut) {
        if (intakeOut) {
            setVIntakeSpeed(-CHConstants.verticalIntakeSpeed);
            setHIntakeSpeed(CHConstants.horizontalIntakeSpeed); // horizontal intake is in reverse
        } else if (sensoredIn && !intakeOut) {
            if (getIntakeSensor()) {
                setVIntakeSpeed(0);
                setHIntakeSpeed(0);
                setLeds(true);
            } else {
                setLeds(false);
                setVIntakeSpeed(CHConstants.verticalIntakeSpeed); // might need to reverse direction
                setHIntakeSpeed(-CHConstants.horizontalIntakeSpeed);
            }
        } 
          else if (intakeIn && !intakeOut){
            setVIntakeSpeed(CHConstants.verticalIntakeSpeed); // might need to reverse direction
            setHIntakeSpeed(-CHConstants.horizontalIntakeSpeed);
        }
          else {
            setLeds(false);
            setHIntakeSpeed(0);
            setVIntakeSpeed(0);
        }
        
        
    }

    public void TTRightLeft(boolean left, boolean right) {
        if (left) {
            setTTableSpeed(CHConstants.turnTableSpeed);
        } else if (right) {
            if (getRightBowlSensor() && getLeftBowlSensor()) {
                setTTableSpeed(0);
            } else {
                setTTableSpeed(-CHConstants.turnTableSpeed);
            }
        } else {
            setTTableSpeed(0);
        }
    }

    private void updateStatus() {
        SmartDashboard.putBoolean("Intake Sensor", getIntakeSensor());
        SmartDashboard.putBoolean("Right Bowl Sensor", getRightBowlSensor());
        SmartDashboard.putBoolean("Left Bowl Sensor", getLeftBowlSensor());
    }

}

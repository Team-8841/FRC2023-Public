package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DSConstants;
import frc.robot.subsystems.Arm;

public class ArmControl extends CommandBase {

    private Arm m_arm;

    private double m_armPos;

    private boolean m_pivot, m_extend, m_gripIn, m_gripOut;

    public ArmControl(double armPos, Boolean pivot, Boolean extend, boolean gripIn, boolean gripOut, Arm arm) {
        m_arm = arm;
        m_armPos = armPos;
        m_pivot = pivot;
        m_extend = extend;
        m_gripIn = gripIn;
        m_gripOut = gripOut;

        addRequirements(arm);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_arm.setArmPivot(m_pivot);
        m_arm.SensorControl(0, m_armPos, false);
        m_arm.grabSensorControl(m_extend, m_gripIn, m_gripOut);

    }

    @Override
    public void end(boolean interrupted) {
        m_arm.setArmPivot(false);
        m_arm.SensorControl(0, DSConstants.posHome, false);
        m_arm.grabSensorControl(false, false, false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}

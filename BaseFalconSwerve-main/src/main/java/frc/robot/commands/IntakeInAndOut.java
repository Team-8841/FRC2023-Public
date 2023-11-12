package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CargoHandler;

public class IntakeInAndOut extends CommandBase {

    private CargoHandler m_CargoHandler;

    private boolean m_in, m_out, m_inSensor;

    public IntakeInAndOut(boolean inSensor, boolean in, Boolean out, CargoHandler cargoHandler) {
        m_CargoHandler = cargoHandler;
        m_in = in;
        m_out = out;
        m_inSensor = inSensor;

        addRequirements(cargoHandler);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_CargoHandler.intakeInAndOut(m_inSensor, m_in, m_out);
    }

    @Override
    public void end(boolean interrupted) {
        m_CargoHandler.intakeInAndOut(false, false, false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}

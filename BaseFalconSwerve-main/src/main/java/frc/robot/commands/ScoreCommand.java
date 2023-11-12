package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;

public class ScoreCommand extends SequentialCommandGroup {



    public ScoreCommand(Arm arm) {
        super(
                new SequentialCommandGroup(
                    new ParallelRaceGroup(
                        new ArmControl(ArmConstants.topConeDrop, true, true, true, false, arm),
                        new WaitCommand(0.3)
                    ),
                    new ParallelRaceGroup(
                        new ArmControl(ArmConstants.topConeDrop, true, false, false, true, arm),
                        new WaitCommand(0.3)
                    ),
                    new ParallelRaceGroup(
                        new ArmControl(ArmConstants.topConeDrop, false, false, false, false, arm),
                        new WaitCommand(0.3)
                    ),
                    new ParallelRaceGroup(
                        new ArmControl(ArmConstants.home, false, false, false, false, arm),
                        new WaitCommand(0.3)
                    )
                )
            );
    }

}

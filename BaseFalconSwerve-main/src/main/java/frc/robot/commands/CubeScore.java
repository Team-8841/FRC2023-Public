package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DSConstants;
import frc.robot.subsystems.Arm;

public class CubeScore extends SequentialCommandGroup {

    public CubeScore(Arm arm) {
        super(
                new SequentialCommandGroup( // Update with arm pos constants
                        //new ParallelRaceGroup(
                         //       new ArmControl(DSConstants.home, false, true, ArmConstants.gripperMotorInSpeed, arm),
                          //      new WaitCommand(1)),
                        new ParallelRaceGroup(
                                new ArmControl(ArmConstants.home, false, false, true, false, arm), //home
                                new WaitCommand(1)),
                        new ParallelRaceGroup(
                                new ArmControl(ArmConstants.top, false, false, true, false, arm), //middle
                                new WaitCommand(2)),
                        new ParallelRaceGroup(
                                new ArmControl(ArmConstants.top, false, true, true, false, arm), //middle
                                new WaitCommand(1)),
                        new ParallelRaceGroup(
                                new ArmControl(ArmConstants.top, false, true, false, true, arm),//middle
                                new WaitCommand(1)),
                        new ParallelRaceGroup(
                                new ArmControl(ArmConstants.top, false, false, false, false, arm), //middle
                                new WaitCommand(1)),
                        new ParallelRaceGroup( 
                                new ArmControl(ArmConstants.home, false, false, false, false, arm),
                                new WaitCommand(1)), //home
                        new WaitCommand(1)
                )
        );
    }

}

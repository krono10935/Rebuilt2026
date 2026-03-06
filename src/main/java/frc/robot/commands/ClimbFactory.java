package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbConstants;

public class ClimbFactory {
    public static Command openClimbWithErrorHandling(Climb climb){
        return new ParallelRaceGroup(
            climb.openCommand(),
            new WaitCommand(ClimbConstants.TIME_FOR_CLIMB_TO_CLOSE_OR_OPEN_CLIMB)
        );
    }
}

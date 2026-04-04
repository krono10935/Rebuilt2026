package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.intake.Intake;

public class CloseSlowAndThenFast{
    public static Command factory(Intake intake){
        return new WaitCommand(1).andThen(new SlowlyCloseOnce(intake), new ShakeItOffCommand(intake));
    }
}

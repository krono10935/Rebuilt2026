package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.intake.Intake;

public class IntakeFactory {

    public static Command resetIntake(Intake intake){
        return Commands.sequence(new InstantCommand(() -> intake.setPositionMotorPercent(-0.4)),
                new WaitCommand(0.3),
                new InstantCommand(() -> {
                    intake.setPositionMotorPercent(0);
                    intake.resetEncoder();
                }));
    }
}

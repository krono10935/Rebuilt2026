// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.utils.ParallelRaceGroupWithWinner;

public class CloseCommand extends Command {
    private Intake intake;

    public CloseCommand(Intake intake) {

    this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.stopIntakeMotor();
        intake.setPosition(IntakeConstants.CLOSE_POSITION);
    }

    @Override
    public boolean isFinished(){
    return intake.positionAtSetPoint();
    }

    public static Command closeWithErrorHandeling(Intake intake){
        @SuppressWarnings("resource")
        Alert closeFailed = new Alert("failed to close intake", AlertType.kError);
        
        Command closeCommandWithErrorHandling = new CloseCommand(intake)
            .andThen(new InstantCommand(() -> closeFailed.set(false)));
        
        Command retryCloseCommandWithErrorHandling = new CloseCommand(intake)
            .andThen(new InstantCommand(() -> closeFailed.set(false)));

        return ParallelRaceGroupWithWinner.andThenOnlyIfTimeout(
            closeCommandWithErrorHandling,
            IntakeConstants.TIME_FOR_INTAKE_TO_CLOSE,

            ParallelRaceGroupWithWinner.andThenOnlyIfTimeout(
                new OpenCommand(intake).andThen(retryCloseCommandWithErrorHandling),
                IntakeConstants.TIME_FOR_INTAKE_TO_OPEN + IntakeConstants.TIME_FOR_INTAKE_TO_CLOSE,
                
                new OpenCommand(intake).andThen(new InstantCommand(() -> closeFailed.set(true)))
            )
        );
    }
}





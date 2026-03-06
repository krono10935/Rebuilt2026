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


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class OpenCommand extends Command {
    private final Intake intake;

    public OpenCommand(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    addRequirements(intake);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    intake.setPosition(IntakeConstants.OPEN_POSITION);
    }

    @Override
    public boolean isFinished(){
    return intake.positionAtSetPoint();
    }

    @Override
    public void end(boolean interrupted){
        if (interrupted) {
            intake.stopIntakeOpeningMotor();
        }
    }

    public static Command openWithErrorHandeling(Intake intake){
        @SuppressWarnings("resource")
        Alert openFailed = new Alert("failed to open intake", AlertType.kError);
        
        Command openCommandWithErrorHandling = new OpenCommand(intake)
            .andThen(new InstantCommand(() -> openFailed.set(false)));

        Command retryOpenCommandWithErrorHandling = new OpenCommand(intake)
            .andThen(new InstantCommand(() -> openFailed.set(false)));

        return ParallelRaceGroupWithWinner.andThenOnlyIfTimeout(
            openCommandWithErrorHandling,
            IntakeConstants.TIME_FOR_INTAKE_TO_OPEN,

            ParallelRaceGroupWithWinner.andThenOnlyIfTimeout(
                new CloseCommand(intake).andThen(retryOpenCommandWithErrorHandling),
                IntakeConstants.TIME_FOR_INTAKE_TO_OPEN + IntakeConstants.TIME_FOR_INTAKE_TO_CLOSE,
                
                new CloseCommand(intake).andThen(new InstantCommand(() -> openFailed.set(true)))
            )
        );
    }
}


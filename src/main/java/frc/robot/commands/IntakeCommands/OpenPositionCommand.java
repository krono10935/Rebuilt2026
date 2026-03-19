// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.utils.ParallelRaceGroupWithWinner;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class OpenPositionCommand extends Command {
    private final Intake intake;
    private final LoggedNetworkNumber setpoint;

    public OpenPositionCommand(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    setpoint = new LoggedNetworkNumber("Open/pos", 0.15);
    addRequirements(intake);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("init open\n skebob");
    intake.setPosition(setpoint.getAsDouble());
    }

    @Override
    public boolean isFinished(){
    return intake.positionAtSetPoint();
    }

    @Override
    public void end(boolean interrupted){
                System.out.println("end");

        if (interrupted) {
                    System.out.println("bad end");

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


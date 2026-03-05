// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;

import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;


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
    return new ParallelRaceGroup(new OpenCommand(intake),
     new IntakeTimeOut(intake, IntakeConstants.TIME_FOR_INTAKE_TO_OPEN, true));
    }
}


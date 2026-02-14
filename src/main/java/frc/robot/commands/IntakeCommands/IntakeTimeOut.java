// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;


import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.intake.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeTimeOut extends WaitCommand {
  private final Intake intake;
    private final boolean forOpen;
    public IntakeTimeOut(Intake intake ,double time, boolean forOpen){
        super(time);
        this.intake = intake;
        this.forOpen = forOpen;
    }

    @Override
    public void end(boolean interrupted){
        super.end(interrupted);
        intake.send(!interrupted, forOpen ? 1 : 0);
    }
}

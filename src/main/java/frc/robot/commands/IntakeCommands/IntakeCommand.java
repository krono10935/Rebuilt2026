// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCommand extends Command {
  /** Creates a new IntakeCommand. */
  private Intake intake;
  private int balls;
  public IntakeCommand(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.balls = 0;
    addRequirements(intake);
  }

  @Override
  public void initialize(){
    intake.startIntakeMotor();
  }




  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double actualPower = intake.getPower();
  }


}

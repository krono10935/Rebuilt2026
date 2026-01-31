// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCommand extends Command {
  /** Creates a new IntakeCommand. */
  private Intake intake;
  public IntakeCommand(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize(){
    intake.startIntake();
  }




  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double actualPower = intake.getPower() - IntakeConstants.IDLE_POWER;
    double totalEnergy = actualPower * 0.02;

    intake.addBalls(ballsAddition(totalEnergy));
  }

  public int ballsAddition(double energy){
    if(energy < IntakeConstants.BALL_INTAKE_ENERGY){
      return 0;
    }

    else{
      return ballsAddition(energy - IntakeConstants.BALL_INTAKE_ENERGY) + 1;
    }
  }

}

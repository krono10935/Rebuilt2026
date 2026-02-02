// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;

/**
 * the intake command
 */
public class IntakeCommand extends Command {
  /** Creates a new IntakeCommand. */
  private Intake intake;
  private double powerSum;
  private double energy;

  public IntakeCommand(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    addRequirements(intake);
    energy = 0;
    powerSum = 0;
  }

  @Override
  public void initialize(){
    intake.setIntakeMotorVelocity(IntakeConstants.INTAKE_VELOCITY);
  }

  /**
   * by knowing the energy it takes for a single ball to be intaked, the motor's energy is known by the sum of the intake motor's power over time and then subtracting the balls already in the intake (previous energy invested) from the total energy. then the amount of balls added to the ballsCounter is the total energy devided by the energy per ball required.
   */
  @Override
  public void execute() {
    powerSum+=intake.getPower() - IntakeConstants.IDLE_POWER;
    
    if(intake.getPower() >= IntakeConstants.INTAKE_POWER_BALL_COUNTER_DEADBAND){
      energy = powerSum*0.02;
      energy -= intake.getBalls()*IntakeConstants.BALL_INTAKE_ENERGY;
      intake.addBalls((int)(energy/IntakeConstants.BALL_INTAKE_ENERGY));
    }
  }

  @Override
  public void end(boolean interrupted){
    intake.stopIntakeMotor();
  }

}

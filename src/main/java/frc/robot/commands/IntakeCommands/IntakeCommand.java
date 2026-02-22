// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;

/**
 * the intake command
 */
public class IntakeCommand extends Command {
  /** Creates a new IntakeCommand. */
  private final Intake intake;
  private final Timer hasBallTimer = new Timer();


  public IntakeCommand(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize(){
    intake.setIntakeMotorVelocity(IntakeConstants.INTAKE_VELOCITY);
    hasBallTimer.reset();
    hasBallTimer.start();
    Shuffleboard.selectTab("Intake Camera");
  }

  /**
   * by knowing the energy it takes for a single ball to be intaked, the motor's energy is known by the sum of the intake motor's power over time and then subtracting the balls already in the intake (previous energy invested) from the total energy. then the amount of balls added to the ballsCounter is the total energy devided by the energy per ball required.
   */
  @Override
  public void execute() {
      if(hasBallTimer.get() >= IntakeConstants.TIME_FOR_BALL_TO_BE_INTAKED ) intake.setHasBalls(true);
  }

  public int addBallsFromEnergy(double energy){
    if(energy < IntakeConstants.BALL_INTAKE_ENERGY){
      return 0;
    }
    else{
      return addBallsFromEnergy(energy - IntakeConstants.BALL_INTAKE_ENERGY) + 1;
    }

  }

  @Override
  public void end(boolean interrupted){
      intake.stopIntakeMotor();
      hasBallTimer.stop();
  }


}

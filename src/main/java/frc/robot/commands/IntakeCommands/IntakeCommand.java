// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.utils.Elastic;

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
    Elastic.selectTab("Intake Camera");
  }

  @Override
  public void end(boolean interrupted){
      intake.stopIntakeMotor();
      hasBallTimer.stop();
  }


}

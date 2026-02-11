// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;

public class CloseCommand extends Command {

  private Intake intake;
  
  public CloseCommand(Intake intake) {

    this.intake = intake;
    addRequirements(intake);

  }
   public static Command closeWithErrorHandeling(Intake intake){
    return new ParallelRaceGroup(new CloseCommand(intake),new WaitCommand(4));
  }

  public static class IntakeTimeOut extends WaitCommand{
    public IntakeTimeOut(double time){
      super(time);
    }

    @Override
    public void end(boolean interrupted){
      super.end(interrupted);
      if(!interrupted){
        //TODO:error
      }
    }
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
}



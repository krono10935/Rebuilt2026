// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;

public class CloseCommand extends Command {

  private Intake intake;
  private int counter = 0;
  private static boolean interruped = false;
  
  public CloseCommand(Intake intake) {

    this.intake = intake;
    addRequirements(intake);

  }
    @Override
    public void execute() {
        //if after 50 execute cycles the intake is not at the open position, it is likely that the intake is stuck on something and the command should be interrupted to prevent damage to the motor
        if(counter<50 || intake.getIntakePosition() >= IntakeConstants.OPEN_POSITION-IntakeConstants.POSITION_TOLERANCE){
            counter++;
        }
        else{
            interruped = true;
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

  public static boolean getInterruped(){
      return interruped;
  }
}

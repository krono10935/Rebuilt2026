// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;

public class CloseCommand extends Command {

  private Intake intake;
  
  public CloseCommand(Intake intake) {

    this.intake = intake;
    addRequirements(intake);

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

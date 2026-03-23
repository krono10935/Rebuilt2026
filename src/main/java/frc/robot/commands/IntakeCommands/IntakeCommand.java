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
 * Command to run the intake mechanism at a constant velocity.
 * Starts a timer to track possession of a ball and selects the intake camera tab.
 */
public class IntakeCommand extends Command {

    private final Intake intake;
    private final Timer hasBallTimer = new Timer();
    
    /**
     * Creates a new IntakeCommand.
     *
     * @param intake The intake subsystem this command controls.
     */
    public IntakeCommand(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    /**
     * 
     * @return if the intake roller is stuck
     */
    public boolean isStuck(){
        return intake.getIntakeMotorCurrent() > IntakeConstants.MAX_INTAKE_CURRENT_TOLERANCE;
    }

    @Override
    public void initialize(){
      intake.setIntake90PercentSpeed(IntakeConstants.INTAKE_VELOCITY);
      hasBallTimer.reset();
      hasBallTimer.start();
      if(isStuck()){
        intake.setIntake90PercentSpeed(IntakeConstants.INTAKE_VELOCITY * (-1));
      }
      Elastic.selectTab("Intake Camera");
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopIntakeMotor();
        hasBallTimer.stop();
    }
}
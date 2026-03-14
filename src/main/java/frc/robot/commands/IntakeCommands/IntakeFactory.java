package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants.ResetConstants;;

public class IntakeFactory {

    public static Command resetIntake(Intake intake){
        return Commands.sequence(new InstantCommand(() -> intake.setPositionMotorPercent(ResetConstants.INITIAL_DUTY_CYCLE_CHECK_FOR_CLOSE)),
                new WaitUntilCommand(() -> intake.getPositionMotorCurrent() > ResetConstants.INITIAL_CURRENT_CHECK_FOR_CLOSE),
                new InstantCommand(() -> {
                    intake.stopIntakeOpeningMotor();
                    intake.resetEncoderOpen(0);
                }),
                new InstantCommand(() -> intake.setPosition(ResetConstants.FINAL_POSITION_CHECK_FOR_CLOSE)),
                new WaitUntilCommand(intake::positionAtSetPoint),
                new InstantCommand(() -> intake.setPositionMotorPercent(ResetConstants.FINAL_DUTY_CYCLE_CHECK_FOR_CLOSE)),
                new WaitUntilCommand(() -> intake.getPositionMotorCurrent() > ResetConstants.FINAL_CURRENT_CHECK_FOR_CLOSE),
                new InstantCommand(() -> {
                    intake.stopIntakeOpeningMotor();
                    intake.resetEncoderOpen(0);
                }));
        } 
}

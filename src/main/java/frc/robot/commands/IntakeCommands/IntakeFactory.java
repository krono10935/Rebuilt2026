package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants.ResetConstants;
import org.littletonrobotics.junction.Logger;

/**
 * Factory class for generating intake-related commands.
 */
public class IntakeFactory {
    public static Command resetIntake(Intake intake) {
        FunctionalCommand initialReset = new FunctionalCommand(
                () -> {
                    intake.setPositionMotorPercent(ResetConstants.INITIAL_DUTY_CYCLE_CHECK_FOR_CLOSE);
                    Logger.recordOutput("intake/reset state", "waiting for initial current");
                },
                () -> {
                },
                (interrupted) -> {
                    intake.stopIntakeOpeningMotor();
                    intake.resetEncoderOpen(0);
                    Logger.recordOutput("intake/reset state", "found for initial current");
                },
                () -> intake.getPositionMotorCurrent() > ResetConstants.INITIAL_CURRENT_CHECK_FOR_CLOSE &&
                        intake.getPositionMotorCurrent() < ResetConstants.MAX_CURRENT_LIMIT,
                intake
        );

        FunctionalCommand moveBack = new FunctionalCommand(
                () -> {
                    intake.setPosition(ResetConstants.FINAL_POSITION_CHECK_FOR_CLOSE);
                    Logger.recordOutput("intake/reset state", "started moving back");
                },
                () -> {
                },
                (interrupted) -> Logger.recordOutput("intake/reset state", "stopped moving back")
                ,
                () -> intake.getIntakePosition() >= ResetConstants.FINAL_POSITION_CHECK_FOR_CLOSE - 0.01
        );

        FunctionalCommand secondReset = new FunctionalCommand(
                () -> {
                    intake.setPositionMotorPercent(ResetConstants.FINAL_DUTY_CYCLE_CHECK_FOR_CLOSE);
                    Logger.recordOutput("intake/reset state", "waiting for second current");
                },
                () -> {
                },
                (interrupted) -> {
                    intake.stopIntakeOpeningMotor();
                    intake.resetEncoderOpen(0);
                    intake.setPosition(0);
                    Logger.recordOutput("intake/reset state", "found for second current");
                },
                () -> intake.getPositionMotorCurrent() > ResetConstants.FINAL_CURRENT_CHECK_FOR_CLOSE &&
                        intake.getPositionMotorCurrent() < ResetConstants.MAX_CURRENT_LIMIT
        );

        return Commands.sequence(initialReset, moveBack, secondReset);
    }
}
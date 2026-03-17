package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants.ResetConstants;
import org.littletonrobotics.junction.Logger;

/**
 * Factory class for generating intake-related commands.
 */
public class IntakeFactory {

    /**
     * Returns a command sequence that resets the intake mechanism.
     *
     * @param intake The intake subsystem to reset.
     * @return A command sequence that performs the intake reset procedure.
     */
    public static Command resetIntake(Intake intake) {
        Command reset = Commands.sequence(
                new InstantCommand(() -> intake.setPositionMotorPercent(ResetConstants.INITIAL_DUTY_CYCLE_CHECK_FOR_CLOSE)),
                // new WaitCommand(0.04), // optional wait, currently commented out
                new InstantCommand(() -> Logger.recordOutput("intake/reset state", "waiting for initial current")),
                new WaitUntilCommand(() ->
                        intake.getPositionMotorCurrent() > ResetConstants.INITIAL_CURRENT_CHECK_FOR_CLOSE &&
                                intake.getPositionMotorCurrent() < 50
                ),
                new InstantCommand(() -> Logger.recordOutput("intake/reset state", "found for initial current")),
                new InstantCommand(() -> {
                    intake.stopIntakeOpeningMotor();
                    intake.resetEncoderOpen(0);
                }),
                new InstantCommand(() -> intake.setPosition(ResetConstants.FINAL_POSITION_CHECK_FOR_CLOSE)),
                new WaitUntilCommand(() ->
                        intake.getIntakePosition() >= ResetConstants.FINAL_POSITION_CHECK_FOR_CLOSE - 0.01
                ),
                new InstantCommand(() -> intake.setPositionMotorPercent(ResetConstants.FINAL_DUTY_CYCLE_CHECK_FOR_CLOSE)),
                // new WaitCommand(0.04), // optional wait, currently commented out
                new InstantCommand(() -> Logger.recordOutput("intake/reset state", "waiting for second current")),
                new WaitUntilCommand(() ->
                        intake.getPositionMotorCurrent() > ResetConstants.FINAL_CURRENT_CHECK_FOR_CLOSE &&
                                intake.getPositionMotorCurrent() < 50
                ),
                new InstantCommand(() -> Logger.recordOutput("intake/reset state", "found for second current")),
                new InstantCommand(() -> {
                    intake.stopIntakeOpeningMotor();
                    intake.resetEncoderOpen(0);
                })
        );

        reset.addRequirements(intake);
        return reset;
    }
}
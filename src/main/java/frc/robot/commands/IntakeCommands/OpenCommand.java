package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.utils.ParallelRaceGroupWithWinner;

/**
 * Command to open the intake mechanism.
 */
public class OpenCommand extends Command {

    private final Intake intake;

    /**
     * Creates a new OpenCommand.
     *
     * @param intake The intake subsystem this command controls.
     */
    public OpenCommand(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setPosition(IntakeConstants.OPEN_POSITION);
    }

    @Override
    public boolean isFinished() {
        return intake.positionAtSetPoint();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            intake.stopIntakeOpeningMotor();
        }
    }

    /**
     * Returns a command that opens the intake with error handling.
     *
     * @param intake The intake subsystem to control.
     * @return A command that opens the intake with retries and alerts if it fails.
     */
    public static Command openWithErrorHandeling(Intake intake) {
        @SuppressWarnings("resource")
        Alert openFailed = new Alert("failed to open intake", AlertType.kError);

        Command openCommandWithErrorHandling = new OpenCommand(intake)
                .andThen(new InstantCommand(() -> openFailed.set(false)));

        Command retryOpenCommandWithErrorHandling = new OpenCommand(intake)
                .andThen(new InstantCommand(() -> openFailed.set(false)));

        return ParallelRaceGroupWithWinner.andThenOnlyIfTimeout(
                openCommandWithErrorHandling,
                IntakeConstants.TIME_FOR_INTAKE_TO_OPEN,

                ParallelRaceGroupWithWinner.andThenOnlyIfTimeout(
                        new CloseCommand(intake).andThen(retryOpenCommandWithErrorHandling),
                        IntakeConstants.TIME_FOR_INTAKE_TO_OPEN + IntakeConstants.TIME_FOR_INTAKE_TO_CLOSE,

                        new CloseCommand(intake).andThen(new InstantCommand(() -> openFailed.set(true)))
                )
        );
    }
}
package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.IntakeCommands.CloseCommand;
import frc.robot.commands.IntakeCommands.IntakeCommand;
import frc.robot.commands.IntakeCommands.OpenCommand;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.intake.Intake;


public class Sequences {



    /**
     * Utility class containing climb and declimb command factories.
     *
     * <p>All commands coordinate multiple subsystems such as Intake,
     * Indexer, Climb, Shooter, and Drivetrain in order to safely
     * execute climb and declimb sequences either manually or
     * autonomously.</p>
     */
    public class ClimbCommands {

        /**
         * Closes and stops all relevant subsystems in parallel,
         * then performs an error check sequence.
         *
         * @param intake    the intake subsystem
         * @param indexer   the indexer subsystem
         * @param climb     the climb subsystem
         * @param shooter   the shooter subsystem
         * @return command that closes all subsystems and checks for errors
         */
        private static Command closeSubsystems(
                Intake intake,
                Indexer indexer,
                Climb climb,
                Shooter shooter
        ) {
            ParallelCommandGroup closeSubsystemsParallel = new ParallelCommandGroup(
                    stopIntakeAndClose(intake),
                    indexer.turnOffIndexerCommand(),
                    new InstantCommand(shooter::stopFlyWheel),
                    climb.closeCommand()
            );

            SequentialCommandGroup checkForErrors = new SequentialCommandGroup();
            checkForErrors.addCommands(new WaitCommand(1));
            checkForErrors.addCommands(
                    new InstantCommand(
                            () -> closeSubsystemsParallel
                                    .getRequirements()
                                    .forEach((requirement) -> {
                                        if (requirement.getCurrentCommand() != null)
                                            SmartDashboard.putString(
                                                    requirement.getName(),
                                                    " has Error when closing"
                                            );
                                    })
                    )
            );

            SequentialCommandGroup closeSubystems = new SequentialCommandGroup(
                    closeSubsystemsParallel,
                    checkForErrors
            );

            return closeSubystems;
        }

        /**
         * Manual climb sequence.
         *
         * <p>Closes all subsystems, waits until the drivetrain
         * reaches a target distance, then opens the climb mechanism.</p>
         *
         * @param intake      the intake subsystem
         * @param indexer     the indexer subsystem
         * @param drivetrain  the drivetrain subsystem
         * @param climb       the climb subsystem
         * @param shooter     the shooter subsystem
         * @return manual climb command
         */
        public static Command manualClimb(
                Intake intake,
                Indexer indexer,
                Drivetrain drivetrain,
                Climb climb,
                Shooter shooter
        ) {
            SequentialCommandGroup climbCommand = new SequentialCommandGroup(
                    closeSubsystems(intake, indexer, climb, shooter),
                    new FunctionalCommand(
                            () -> {},
                            () -> {},
                            (isFinished) -> {},
                            () -> drivetrain
                                    .getEstimatedPosition()
                                    .getTranslation()
                                    .getDistance(new Translation2d(1, 1)) < 3.5
                    ),
                    climb.openCommand()
            );

            return climbCommand;
        }

        /**
         * Manual declimb sequence.
         *
         * <p>Closes the climb, performs validation checks,
         * and retries open/close if an error is detected.</p>
         *
         * @param intake   the intake subsystem
         * @param indexer  the indexer subsystem
         * @param climb    the climb subsystem
         * @param shooter  the shooter subsystem
         * @return manual declimb command
         */
        public static Command manualDeclimb(
                Intake intake,
                Indexer indexer,
                Climb climb,
                Shooter shooter
        ) {
            SequentialCommandGroup declimbCommand = new SequentialCommandGroup();

            declimbCommand.addCommands(climb.closeCommand());
            declimbCommand.addCommands(new WaitCommand(1));

            declimbCommand.addCommands(
                    new InstantCommand(
                            () -> declimbCommand
                                    .getRequirements()
                                    .forEach((requirement) -> {
                                        if (requirement.getCurrentCommand() != null) {
                                            declimbCommand.addCommands(climb.openCommand());
                                            declimbCommand.addCommands(climb.closeCommand());
                                        }
                                    })
                    )
            );

            declimbCommand.addCommands(new WaitCommand(1));

            declimbCommand.addCommands(
                    new InstantCommand(
                            () -> declimbCommand
                                    .getRequirements()
                                    .forEach((requirement) -> {
                                        if (requirement.getCurrentCommand() != null) {
                                            SmartDashboard.putString(
                                                    requirement.getName(),
                                                    " has Error when declimbing"
                                            );
                                        }
                                    })
                    )
            );

            return declimbCommand;
        }

        /**
         * Autonomous climb sequence.
         *
         * <p>Drives to a target pose while closing subsystems,
         * opens the climb, then performs validation checks.
         * Only runs during endgame.</p>
         *
         * @param intake      the intake subsystem
         * @param indexer     the indexer subsystem
         * @param drivetrain  the drivetrain subsystem
         * @param climb       the climb subsystem
         * @param shooter     the shooter subsystem
         * @return autonomous climb command
         */
        public static Command autoClimb(
                Intake intake,
                Indexer indexer,
                Drivetrain drivetrain,
                Climb climb,
                Shooter shooter
        ) {

            SequentialCommandGroup climbCommand = new SequentialCommandGroup();
            ParallelCommandGroup ParallelClimbCommand = new ParallelCommandGroup();

            SequentialCommandGroup sequentialClimbCommandGroup = new SequentialCommandGroup(
                    closeSubsystems(intake, indexer, climb, shooter),
                    new FunctionalCommand(
                            () -> {},
                            () -> {},
                            (isFinished) -> {},
                            () -> drivetrain
                                    .getEstimatedPosition()
                                    .getTranslation()
                                    .getDistance(new Translation2d(1, 1)) < 3.5
                    ),
                    climb.openCommand()
            );

            sequentialClimbCommandGroup.addCommands(
                    new InstantCommand(
                            () -> sequentialClimbCommandGroup
                                    .getRequirements()
                                    .forEach((requirement) -> {
                                        if (requirement.getCurrentCommand() != null) {
                                            SmartDashboard.putString(
                                                    requirement.getName(),
                                                    " has Error when climbing"
                                            );
                                        }
                                    })
                    )
            );

            ParallelClimbCommand.addCommands(
                    drivetrain.driveToPose(
                            new Pose2d(1, 1, Rotation2d.fromDegrees(1))
                    )
            );

            ParallelClimbCommand.addCommands(sequentialClimbCommandGroup);

            climbCommand.addCommands(ParallelClimbCommand);
            climbCommand.addCommands(climb.closeCommand());

            climbCommand.addCommands(
                    new InstantCommand(
                            () -> climbCommand
                                    .getRequirements()
                                    .forEach((requirement) -> {
                                        if (requirement.getCurrentCommand() != null) {
                                            SmartDashboard.putString(
                                                    requirement.getName(),
                                                    " has Error when climbing"
                                            );
                                        }
                                    })
                    )
            );

            if (DriverStation.getMatchTime() > 140)
                climb.setHasClimbed(true);

            return climbCommand.onlyIf(() -> DriverStation.getMatchTime() > 140);
        }

        /**
         * Autonomous declimb sequence.
         *
         * <p>Opens the climb, drives away from the bar,
         * then closes the climb with validation checks.</p>
         *
         * @param intake      the intake subsystem
         * @param indexer     the indexer subsystem
         * @param drivetrain  the drivetrain subsystem
         * @param climb       the climb subsystem
         * @param shooter     the shooter subsystem
         * @return autonomous declimb command
         */
        public static Command autoDeclimb(
                Intake intake,
                Indexer indexer,
                Drivetrain drivetrain,
                Climb climb,
                Shooter shooter
        ) {
            SequentialCommandGroup declimbCommand = new SequentialCommandGroup();

            declimbCommand.addCommands(climb.openCommand());

            declimbCommand.addCommands(
                    drivetrain.driveToPose(
                            new Pose2d(1, 1, Rotation2d.fromDegrees(1))
                    )
            );

            declimbCommand.addCommands(
                    new FunctionalCommand(
                            () -> {},
                            () -> {},
                            (isFinished) -> {},
                            () -> drivetrain
                                    .getEstimatedPosition()
                                    .getTranslation()
                                    .getDistance(new Translation2d(1, 1)) > 0.5
                    )
            );

            declimbCommand.addCommands(climb.closeCommand());

            declimbCommand.addCommands(
                    new InstantCommand(
                            () -> declimbCommand
                                    .getRequirements()
                                    .forEach((requirement) -> {
                                        if (requirement.getCurrentCommand() != null) {
                                            SmartDashboard.putString(
                                                    requirement.getName(),
                                                    " has Error when climbing"
                                            );
                                            declimbCommand.addCommands(climb.openCommand());
                                            declimbCommand.addCommands(climb.closeCommand());
                                        }
                                    })
                    )
            );

            declimbCommand.addCommands(
                    new InstantCommand(
                            () -> declimbCommand
                                    .getRequirements()
                                    .forEach((requirement) -> {
                                        if (requirement.getCurrentCommand() != null) {
                                            SmartDashboard.putString(
                                                    requirement.getName(),
                                                    " has Error when climbing"
                                            );
                                        }
                                    })
                    )
            );

            return declimbCommand.onlyIf(() -> climb.getHasClimbed());
        }
    }

/**
 * Opens the intake and starts the intake motor.
 * <p>
 * Behavior:
 * <ul>
 *   <li>Attempts to open the intake</li>
 *   <li>If opening succeeds, starts the intake motor</li>
 *   <li>If opening fails, attempts to close the intake as a recovery step</li>
 *   <li>If both opening and closing fail, fully disables the intake subsystem</li>
 * </ul>
 *
 * This command is designed to prevent further damage if the intake
 * mechanism becomes stuck or unreliable.
 *
 * @param intake the intake subsystem
 * @return a SequentialCommandGroup that safely opens and starts the intake
 */
public static Command intakeOpenStart(Intake intake) {

    SequentialCommandGroup intakeCommand = new SequentialCommandGroup();

    // --- Step 1: Attempt to open the intake ---
    intakeCommand.addCommands(
        new OpenCommand(intake)
    );

    // --- If opening was interrupted, attempt recovery ---
    if (OpenCommand.getInterruped()) {

        // Try closing the intake to reset its state
        intakeCommand.addCommands(
            new CloseCommand(intake)
        );

        // --- If closing also fails, disable the intake completely ---
        if (CloseCommand.getInterruped()) {
            shutdownIntake(
                intake,
                "CRITICAL FAILURE🥀, ",
                "Intake failed to open and close. Intake disabled for safety." +
                "it is ok twin ❤️"
            );
            return intakeCommand;
        }

        // Report that the open command failed but recovery succeeded
        SmartDashboard.putBoolean(
            "Intake Open Interrupted",
            true
        );
    }
    else {
        // --- Step 2: Intake opened successfully, start intake motor ---
        intakeCommand.addCommands(
            new IntakeCommand(intake)
        );
    }

    return intakeCommand;
}

    /**
 * Stops the intake motor and attempts to safely close the intake.
 * <p>
 * Behavior:
 * <ul>
 *   <li>Stops the intake motor immediately</li>
 *   <li>Attempts to close the intake</li>
 *   <li>If closing fails, attempts recovery by reopening, reversing, and retrying</li>
 *   <li>If all recovery attempts fail, fully disables the intake subsystem</li>
 * </ul>
 *
 * This command is designed to be fail-safe: if the intake mechanism
 * cannot move reliably, it is shut down to prevent hardware damage.
 *
 * @param intake the intake subsystem
 * @return a SequentialCommandGroup that performs the stop-and-close logic
 */
public static Command stopIntakeAndClose(Intake intake) {

    SequentialCommandGroup intakeCommand = new SequentialCommandGroup();

    // --- Step 1: Stop intake roller immediately ---
    intakeCommand.addCommands(
        new InstantCommand(intake::stopIntakeMotor)
    );

    // --- Step 2: Try to close the intake ---
    intakeCommand.addCommands(
        new CloseCommand(intake)
    );

    // --- If closing was interrupted, start recovery procedure ---
    if (CloseCommand.getInterruped()) {

        // Attempt to reopen the intake
        intakeCommand.addCommands(
            new OpenCommand(intake)
        );

        // --- If reopening also fails, disable intake completely ---
        if (OpenCommand.getInterruped()) {
            shutdownIntake(intake,
                "CRITICAL FAILURE🥀, ",
                "Intake failed to close and reopen. Intake has been disabled for safety." +
                "it is ok twin ❤️"
            );
            return intakeCommand;
        }

        // --- Recovery attempt: reverse intake to clear obstruction ---
        intakeCommand.addCommands(
            new InstantCommand(() ->
                intake.setIntakeMotorVelocity(
                    Rotation2d.fromRotations(-2)
                )
            )
        );

        // Allow time for obstruction to clear
        intakeCommand.addCommands(
            new WaitCommand(2)
        );

        // Stop intake roller again
        intakeCommand.addCommands(
            new InstantCommand(intake::stopIntakeMotor)
        );

        // Retry closing intake
        intakeCommand.addCommands(
            new CloseCommand(intake)
        );

        // --- If closing still fails after recovery, shut everything down ---
        if (CloseCommand.getInterruped()) {
            shutdownIntake(intake,
                "CRITICAL FAILURE🥀, ",
                "Intake failed after recovery attempt. Intake has been disabled for safety." +
                "it is ok twin ❤️"
            );
            return intakeCommand;
        }
    }

    return intakeCommand;
}

/**
 * Safely disables the intake subsystem and reports a critical error.
 *
 * @param intake the intake subsystem
 * @param title dashboard error title
 * @param message dashboard error message
 */
private static void shutdownIntake(Intake intake, String title, String message) {

    // Stop all motors
    new InstantCommand(intake::stopIntakeMotor);
    new InstantCommand(() ->
        intake.setPositionMotorPercentOutput(0)
    );

    // Report error clearly to drivers
    SmartDashboard.putString(
        "INTAKE STATUS",
        title + message
    );
}

}
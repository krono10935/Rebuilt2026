package frc.robot;

import com.fasterxml.jackson.databind.deser.std.FromStringDeserializer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveAndHomeCommand;
import frc.robot.commands.IntakeCommands.CloseCommand;
import frc.robot.commands.IntakeCommands.IntakeCommand;
import frc.robot.commands.IntakeCommands.OpenCommand;
import frc.robot.commands.Shooter.ShootCommand;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.intake.Intake;


public class Sequences {
    /**
     * turns on the indexer then aims the hood to the hub according to the robot's position
     * and pew pew
     * @param shooter
     * @param indexer
     * @param drivetrain
     * @return
     */
    public static Command shoot(Shooter shooter, Indexer indexer,
                                Drivetrain drivetrain , CommandXboxController CommandXboxController) {
        SequentialCommandGroup shooterCommand = new SequentialCommandGroup();
        shooterCommand.addCommands(indexer.turnOnIndexerCommand());
        shooterCommand.addCommands
                (ShootCommand.shootCommandFactory(shooter,drivetrain, CommandXboxController).
                        alongWith(
                                new DriveAndHomeCommand(drivetrain,
                                        CommandXboxController)));
        return shooterCommand.withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf);
    }

    /**
     * opens the intake sets the hood to the right angle turns on indexer and pew pew
     * @param shooter
     * @param indexer
     * @param intake
     * @param drivetrain
     * @return
     */
    public static Command delivery(Shooter shooter, Indexer indexer, Intake intake, Drivetrain drivetrain) {
        SequentialCommandGroup deliveryCommand = new SequentialCommandGroup();
      //  deliveryCommand.addCommands(Sequences.openIntakeStart(intake));
        deliveryCommand.addCommands(new InstantCommand(()-> shooter.setHoodAngle(Rotation2d.fromDegrees(45))));
        deliveryCommand.addCommands(indexer.turnOnIndexerCommand());
        // deliveryCommand.addCommands(new ShootCommand(shooter,drivetrain,XboxController)); //TODO make this make sense
        return deliveryCommand.withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf);
    }

    /**
     * Drives near the tower then opens the climbing mechanism moves into the towers
     * rung and closes the climbing mechanism
     * @param climb
     * @param drivetrain
     * @return
     */
    private static Command Climber(Climb climb,Drivetrain drivetrain){
        SequentialCommandGroup climbCommand = new SequentialCommandGroup();

        climbCommand.addCommands(drivetrain.driveToPose(new Pose2d(1,1, Rotation2d.fromDegrees(1))));
        climbCommand.addCommands(climb.openCommand());
        climbCommand.addCommands(drivetrain.driveToPose(new Pose2d(1,1.5, Rotation2d.fromDegrees(1))));
        climbCommand.addCommands(climb.closeCommand());

        return climbCommand;

    }

    /**
     * Closes all the unnecessary subsystem for climbing
     * then runs the Climber sequence
     * (drives near the tower then opens the climbing mechanism moves into the towers
     *  rung and closes the climbing mechanism)
     * @param intake
     * @param climb
     * @param indexer
     * @param shooter
     * @param drivetrain
     * @return
     */
    public static Command FullClimb(Intake intake, Climb climb, Indexer indexer,Shooter shooter, Drivetrain drivetrain) {
        ParallelCommandGroup closeSubsystems = new ParallelCommandGroup(
             //   Sequences.closeIntakeStop(intake),
                indexer.turnOffIndexerCommand(),
                new InstantCommand(shooter::stopFlyWheel)
                
        );


        SequentialCommandGroup fullClimbCommand = new SequentialCommandGroup(
                closeSubsystems,
                Climber(climb,drivetrain)
        );

        return fullClimbCommand;
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
    new InstantCommand(intake::stopIntakeMotor).schedule();
    new InstantCommand(() ->
        intake.setPositionMotorPercentOutput(0)
    ).schedule();

    // Report error clearly to drivers
    SmartDashboard.putString(
        "INTAKE STATUS",
        title + message
    );
}

}
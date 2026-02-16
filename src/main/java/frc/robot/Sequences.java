package frc.robot;

import java.util.Set;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.commands.DriveAndHomeToSupplierCommand;
import frc.robot.commands.SpinUpForDelivery;
import frc.robot.commands.IntakeCommands.CloseCommand;
import frc.robot.commands.IntakeCommands.IntakeCommand;
import frc.robot.commands.IntakeCommands.OpenCommand;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.utils.ErrorMessage;

import static frc.robot.FieldConstants.getTowerSideTargetPose;


public class Sequences {

    /**
     * Checks if any of the given subsystems are still running commands
     * after attempting to close them, and reports errors if needed.
     *
     * @param requirements subsystems to check
     * @return command that sends error messages if closing failed
     */
    public static Command checkForErrorsAfterTryingToClose(Set<Subsystem> requirements) {

        return new InstantCommand(() ->
                requirements.forEach((requirement) -> {
                    ((ErrorMessage.ErrorSender) requirement).send(
                            requirement.getCurrentCommand() != null,0
                    );
                })
        );
    }


    /**
     * Determines whether the robot is close enough to the tower
     * to safely open the climb.
     *
     * @param robotPose current robot pose
     * @return true if close enough to open climb
     */
    private static boolean closeEnoughToOpenClimb(Pose2d robotPose) {

        return robotPose.getTranslation().getDistance(FieldConstants.tower)
                < ClimbConstants.MIN_DISTANCE_FROM_TOWER_TO_OPEN_CLIMB;
    }


    /**
     * Determines whether the robot is far enough from the tower
     * to safely close the climb.
     *
     * @param robotPose current robot pose
     * @return true if far enough to close climb
     */
    private static boolean farEnoughToCloseClimb(Pose2d robotPose) {

        return robotPose.getTranslation().getDistance(FieldConstants.tower)
                >= ClimbConstants.MIN_DISTANCE_FROM_TOWER_TO_CLOSE_CLIMB;
    }


    /**
     * Attempts to close the climb and retries if the first attempt fails.
     * If all retries fail, the climb is disabled and an error is reported.
     *
     * @param climb climb subsystem
     * @return command that closes and retries if needed
     */
    private static Command closeAndRetryClosingIfFailed(Climb climb) {

        SequentialCommandGroup retry = new SequentialCommandGroup(

                new InstantCommand(() -> {
                    if (!climb.isAtSetPoint()) {
                        SmartDashboard.putString(
                                "Climb",
                                "had problems with closing, retrying now"
                        );
                        SmartDashboard.putBoolean("ClimbErrorFatal", false);
                        climb.send(true,0);
                    }
                }),

                climb.openCommand(),

                new WaitCommand(
                        ClimbConstants.TIME_FOR_CLIMB_TO_CLOSE_OR_OPEN_CLIMB
                ),

                climb.closeCommand().withDeadline(
                        new WaitCommand(
                                ClimbConstants.TIME_FOR_CLIMB_TO_CLOSE_OR_OPEN_CLIMB
                        ).withDeadline(
                                new WaitUntilCommand(climb::isAtSetPoint)
                        )
                ),

                new InstantCommand(() -> {
                    if (!climb.isAtSetPoint()) {
                        SmartDashboard.putString(
                                "Climb",
                                "had problems with closing, shutting down"
                        );
                        SmartDashboard.putBoolean("ClimbErrorFatal", true);
                        climb.send(true,0);
                    }
                })
        );


        SequentialCommandGroup closeAndRetryIfFailed =
                new SequentialCommandGroup(

                        climb.closeCommand().withDeadline(
                                new WaitCommand(
                                        ClimbConstants.TIME_FOR_CLIMB_TO_CLOSE_OR_OPEN_CLIMB
                                ).withDeadline(
                                        new WaitUntilCommand(climb::isAtSetPoint)
                                )
                        ),

                        retry.unless(climb::isAtSetPoint)
                );

        return closeAndRetryIfFailed;
    }


    /**
     * Closes and stops all relevant subsystems in parallel,
     * then performs an error check.
     *
     * @param intake  intake subsystem
     * @param climb   climb subsystem
     * @param shooter shooter subsystem
     * @return command that safely closes all subsystems
     */
    private static Command closeSubsystems(
            Intake intake,
            Climb climb,
            Shooter shooter
    ) {

        ParallelCommandGroup closeSubsystemsParallel =
                new ParallelCommandGroup(

                        stopIntakeAndClose(intake),

                        shooter.getIndexer().turnOffIndexerCommand(),

                        new InstantCommand(shooter::stopFlyWheel),

                        new InstantCommand(() ->
                                shooter.toggleKicker(false)
                        ),

                        climb.closeCommand()
                );


        ParallelRaceGroup giveTimeToCloseSubsystems =
                new ParallelRaceGroup(
                        closeSubsystemsParallel,
                        new WaitCommand(
                                Constants.ALL_SUBSYSTEMS_MAX_CLOSING_TIME
                        )
                );


        return giveTimeToCloseSubsystems.andThen(
                checkForErrorsAfterTryingToClose(
                        closeSubsystemsParallel.getRequirements()
                )
        );
    }


    /**
     * Closes all subsystems and then opens the climb.
     *
     * @param intake     intake subsystem
     * @param drivetrain drivetrain subsystem
     * @param climb      climb subsystem
     * @param shooter    shooter subsystem
     * @return command that prepares and opens the climb
     */
    public static Command climbOpen(
            Intake intake,
            Drivetrain drivetrain,
            Climb climb,
            Shooter shooter
    ) {

        return new SequentialCommandGroup(
                closeSubsystems(intake, climb, shooter),
                climb.openCommand()
        );
    }


    /**
     * Attempts to close the climb and checks for errors.
     *
     * @param intake     intake subsystem
     * @param climb      climb subsystem
     * @param shooter    shooter subsystem
     * @param drivetrain drivetrain subsystem
     * @return command that closes the climb
     */
    public static Command climbClose(
            Intake intake,
            Climb climb,
            Shooter shooter,
            Drivetrain drivetrain
    ) {

        SequentialCommandGroup climbCommand =
                new SequentialCommandGroup();

        climbCommand.addCommands(climb.closeCommand());

        climbCommand.addCommands(
                new WaitCommand(
                        ClimbConstants.TIME_FOR_CLIMB_TO_CLOSE_OR_OPEN_CLIMB
                )
        );

        climbCommand.addCommands(
                checkForErrorsAfterTryingToClose(
                        climbCommand.getRequirements()
                )
        );

        return climbCommand;
    }


    /**
     * Autonomous climb sequence.
     *
     * <p>
     * Drives to the tower while closing subsystems,
     * opens the climb, and then verifies successful closure.
     * Only runs during endgame or autonomous.
     * </p>
     *
     * @param intake     intake subsystem
     * @param drivetrain drivetrain subsystem
     * @param climb      climb subsystem
     * @param shooter    shooter subsystem
     * @return autonomous climb command
     */
    public static Command autoClimb(
            Intake intake,
            Drivetrain drivetrain,
            Climb climb,
            Shooter shooter
    ) {

        ParallelCommandGroup parallelClimbCommand =
                new ParallelCommandGroup(
                        closeSubsystems(intake, climb, shooter),

                        Commands.sequence(
                                new WaitUntilCommand(() ->
                                        closeEnoughToOpenClimb(
                                                drivetrain.getEstimatedPosition()
                                        )
                                ),
                                climb.openCommand(

                                )),

                        drivetrain.driveToPose(getTowerSideTargetPose(Constants.chosenTowerSideToClimb, false)
                ) );

        return parallelClimbCommand.onlyIf(() ->
                DriverStation.getMatchTime() > 140
                        || DriverStation.isAutonomous()
        );
    }


    /**
     * Autonomous declimb sequence.
     *
     * <p>
     * Opens the climb, drives away from the tower,
     * and then safely closes the climb.
     * </p>
     *
     * @param intake     intake subsystem
     * @param drivetrain drivetrain subsystem
     * @param climb      climb subsystem
     * @param shooter    shooter subsystem
     * @return autonomous declimb command
     */
    public static Command autoDeclimb(
            Intake intake,
            Drivetrain drivetrain,
            Climb climb,
            Shooter shooter
    ) {

        SequentialCommandGroup mainDeclimbCommand =
                new SequentialCommandGroup();

        mainDeclimbCommand.addCommands(
                climb.openCommand()
        );

        mainDeclimbCommand.addCommands(
                drivetrain.driveToPose(
                        getTowerSideTargetPose(
                                Constants.chosenTowerSideToClimb,
                                true
                        )
                )
        );

        mainDeclimbCommand.addCommands(
                new WaitUntilCommand(() ->
                        farEnoughToCloseClimb(
                                drivetrain.getEstimatedPosition()
                        )
                )
        );

        mainDeclimbCommand.addCommands(
                closeAndRetryClosingIfFailed(climb)
        );

        return mainDeclimbCommand.onlyIf(
                climb::getHasClimbed
        );
    }


    /**
     * Opens the intake and starts the intake motor safely.
     *
     * <p>
     * If opening fails, recovery steps are attempted.
     * If recovery fails, the intake is disabled.
     * </p>
     *
     * @param intake intake subsystem
     * @return command that safely opens and starts intake
     */
    public static Command intakeOpenStart(Intake intake) {

        return OpenCommand.openWithErrorHandeling(intake).
        alongWith(new IntakeCommand(intake));
    }


    /**
     * Stops the intake motor and safely closes the intake.
     *
     * <p>
     * If closing fails, recovery steps are attempted.
     * If all attempts fail, the intake is disabled.
     * </p>
     *
     * @param intake intake subsystem
     * @return command that stops and closes intake
     */
    public static Command stopIntakeAndClose(Intake intake) {

        return new SequentialCommandGroup(

                new InstantCommand(
                        intake::stopIntakeMotor
                ),

                CloseCommand.closeWithErrorHandeling(intake)
        );
    }


    /**
     * Spins up the shooter and delivers a game piece
     * while automatically aligning to the nearest trench.
     *
     * @param drivetrain drivetrain subsystem
     * @param shooter    shooter subsystem
     * @param controller driver controller
     * @return delivery command
     */
    public static Command delivery(
            Drivetrain drivetrain,
            Shooter shooter,
            CommandXboxController controller,
            Intake intake
    ) {

        SequentialCommandGroup spinUpAndAimHood =
                new SequentialCommandGroup();

        spinUpAndAimHood.addCommands(
                new SpinUpForDelivery(
                        drivetrain,
                        shooter,
                        ShooterConstants.DELIVERY_SPEED_MPS
                ).alongWith(new InstantCommand(()->shooter.setHoodAngle(ShooterConstants.DELIVERY_HOOD_ANGLE)))
        );

        Supplier<Translation2d> closestTrenchSupplier =
                () -> (drivetrain.getEstimatedPosition().getY()
                        >= FieldConstants.fieldWidth / 2.0)
                        ? FieldConstants.trenchRight
                        : FieldConstants.trenchLeft;


        Supplier<Rotation2d> deliveryAngleSupplier =
                () -> closestTrenchSupplier.get()
                        .minus(
                                drivetrain.getEstimatedPosition()
                                        .getTranslation()
                        )
                        .getAngle();



        ParallelCommandGroup deliver = new ParallelCommandGroup(
                new RunCommand(() ->
                        shooter.keepVelocity(
                                ShooterConstants.DELIVERY_SPEED_MPS
                        )
                ).onlyIf(() -> intake.hasBalls() && drivetrain.getChassisSpeeds().equals(new ChassisSpeeds())),


                new DriveAndHomeToSupplierCommand(
                        drivetrain,
                        controller,
                        deliveryAngleSupplier
                )

        );

        return spinUpAndAimHood.andThen(deliver);
    }

}

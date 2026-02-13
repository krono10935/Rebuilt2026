package frc.robot;

import java.util.Set;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveAndHomeToSupplierCommand;
import frc.robot.commands.SpinUpForDelivery;
import frc.robot.commands.IntakeCommands.CloseCommand;
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

    public static Command checkForErrorsAfterTryingToClose(Set<Subsystem> requirements){
        return new InstantCommand(() -> requirements.forEach((requirement) -> {
            ((ErrorMessage.ErrorSender)requirement).send(
                    requirement.getCurrentCommand() != null
            );
        }));
    }


    private static boolean closeEnoughToOpenClimb(Pose2d robotPose){
        return robotPose.getTranslation().getDistance(FieldConstants.tower) <
                ClimbConstants.MIN_DISTANCE_FROM_TOWER_TO_OPEN_CLIMB;
    }

    private static boolean farEnoughToCloseClimb(Pose2d robotPose){
        return robotPose.getTranslation().getDistance(FieldConstants.tower) >=
                ClimbConstants.MIN_DISTANCE_FROM_TOWER_TO_CLOSE_CLIMB;
    }

    private static Command CloseAndRetryClosingIfFailed(Climb climb){

        SequentialCommandGroup retry = new SequentialCommandGroup(
                new InstantCommand(() -> {
                    if(!climb.isAtSetPoint()){
                        SmartDashboard.putString("Climb"," had problems with closing retrying now");
                        SmartDashboard.putBoolean("ClimbErrorFatal",false);
                        climb.send(true);
                    }
                }),
                climb.openCommand(),
                new WaitCommand(ClimbConstants.TIME_FOR_CLIMB_TO_CLOSE_OR_OPEN_CLIMB),
                climb.closeCommand().withDeadline(
                        new WaitCommand(ClimbConstants.TIME_FOR_CLIMB_TO_CLOSE_OR_OPEN_CLIMB).
                                withDeadline(new FunctionalCommand(() -> {}, ()-> {},
                                        (interrupted) -> {}, climb::isAtSetPoint)
                                )
                ),
                new InstantCommand(() -> {
                    if(!climb.isAtSetPoint()){
                        SmartDashboard.putString("Climb"," had problems with closing shutting down now");
                        SmartDashboard.putBoolean("ClimbErrorFatal",false);
                        climb.send(true);
                    }
                })
        );

        SequentialCommandGroup closeAndRetyIfFailed = new SequentialCommandGroup(
                climb.closeCommand()
                        .withDeadline(
                                new WaitCommand(ClimbConstants.TIME_FOR_CLIMB_TO_CLOSE_OR_OPEN_CLIMB).
                                        withDeadline(new FunctionalCommand(() -> {}, ()-> {},
                                                (interrupted) -> {}, climb::isAtSetPoint)
                                        )
                        ),
                retry.unless(climb::isAtSetPoint)
        );
        return  closeAndRetyIfFailed;
    }
    /**
     * Closes and stops all relevant subsystems in parallel,
     * then performs an error check sequence.
     *
     * @param intake    the intake subsystem
     * @param climb     the climb subsystem
     * @param shooter   the shooter subsystem
     * @return command that closes all subsystems and checks for errors
     */
    private static Command closeSubsystems(Intake intake, Climb climb, Shooter shooter) {
        ParallelCommandGroup closeSubsystemsParallel = new ParallelCommandGroup(
                stopIntakeAndClose(intake),
                shooter.getIndexer().turnOffIndexerCommand(),
                new InstantCommand(shooter::stopFlyWheel),
                new InstantCommand(() -> shooter.toggleKicker(false)),
                climb.closeCommand()
        );

        ParallelRaceGroup giveTimeToCloseSubsystems =
                new ParallelRaceGroup(closeSubsystemsParallel, new WaitCommand(
                        Constants.ALL_SUBSYSTEMS_MAX_CLOSING_TIME
                ));

        return giveTimeToCloseSubsystems.andThen(
                checkForErrorsAfterTryingToClose(closeSubsystemsParallel.getRequirements()));
    }

    public static Command climbOpen(
            Intake intake,
            Drivetrain drivetrain,
            Climb climb,
            Shooter shooter
    ) {
        SequentialCommandGroup climbCommand = new SequentialCommandGroup(
                closeSubsystems(intake, climb, shooter),
                // add trigger
                climb.openCommand()
        );

        return climbCommand;
    }

    public static Command ClimbClose(
            Intake intake,
            Climb climb,
            Shooter shooter, Drivetrain drivetrain
    ) {
        SequentialCommandGroup climbCommand = new SequentialCommandGroup();

        climbCommand.addCommands(climb.closeCommand());
        climbCommand.addCommands(new WaitCommand(ClimbConstants.TIME_FOR_CLIMB_TO_CLOSE_OR_OPEN_CLIMB));
        climbCommand.addCommands(checkForErrorsAfterTryingToClose(climbCommand.getRequirements()));

        return climbCommand;
    }

    /**
     * Autonomous climb sequence.
     *
     * <p>Drives to a target pose while closing subsystems,
     * opens the climb, then performs validation checks.
     * Only runs during endgame.</p>
     *
     * @param intake      the intake subsystem
     * @param drivetrain  the drivetrain subsystem
     * @param climb       the climb subsystem
     * @param shooter     the shooter subsystem
     * @return autonomous climb command
     */
    public static Command autoClimb(
            Intake intake,
            Drivetrain drivetrain,
            Climb climb,
            Shooter shooter
    ) {

        SequentialCommandGroup mainClimbCommand = new SequentialCommandGroup();
        ParallelCommandGroup ParallelClimbCommand = new ParallelCommandGroup();

        SequentialCommandGroup sequentialClimbCommandGroup = new SequentialCommandGroup(
                closeSubsystems(intake, climb, shooter),
                new WaitUntilCommand(()-> closeEnoughToOpenClimb(drivetrain.getEstimatedPosition())),
                climb.openCommand()
        );

        ParallelClimbCommand.addCommands(
                drivetrain.driveToPose(
                        getTowerSideTargetPose(Constants.chosenTowerSideToClimb,false)
                )
        );

        ParallelClimbCommand.addCommands(sequentialClimbCommandGroup);
        mainClimbCommand.addCommands(ParallelClimbCommand);

        mainClimbCommand.addCommands(CloseAndRetryClosingIfFailed(climb));

        return mainClimbCommand.onlyIf(() -> DriverStation.getMatchTime() > 140 ||
                DriverStation.isAutonomous());
    }

    /**
     * Autonomous declimb sequence.
     *
     * <p>Opens the climb, drives away from the bar,
     * then closes the climb with validation checks.</p>
     *
     * @param intake      the intake subsystem
     * @param drivetrain  the drivetrain subsystem
     * @param climb       the climb subsystem
     * @param shooter     the shooter subsystem
     * @return autonomous declimb command
     */
    public static Command autoDeclimb(
            Intake intake,
            Drivetrain drivetrain,
            Climb climb,
            Shooter shooter
    ) {
        SequentialCommandGroup mainDeclimbCommand = new SequentialCommandGroup();

        mainDeclimbCommand.addCommands(climb.openCommand());

        mainDeclimbCommand.addCommands(
                drivetrain.driveToPose(
                        getTowerSideTargetPose(Constants.chosenTowerSideToClimb,true)
                )
        );

        mainDeclimbCommand.addCommands(new WaitUntilCommand(() ->
                farEnoughToCloseClimb(drivetrain.getEstimatedPosition())));

        mainDeclimbCommand.addCommands(CloseAndRetryClosingIfFailed(climb));

        return mainDeclimbCommand.onlyIf(climb::getHasClimbed);
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

    return OpenCommand.openWithErrorHandeling(intake);
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

    return new SequentialCommandGroup(
        // --- Step 1: Stop intake roller immediately ---
        new InstantCommand(intake::stopIntakeMotor),

        // --- Step 2: Try to close the intake ---
        CloseCommand.closeWithErrorHandeling(intake)
    );
}

    /**
     *
     * @param drivetrain drivetrain
     * @param shooter shooter
     * @param controller controller to drive with
     * @return a command which spinsUp the shooter and then shoots whilst homing robot angle to the closest trench
     */
    public static Command delivery(Drivetrain drivetrain, Shooter shooter, CommandXboxController controller){
    SequentialCommandGroup deliveryGroup = new SequentialCommandGroup();
    deliveryGroup.addCommands(new SpinUpForDelivery(drivetrain, shooter, ShooterConstants.DELIVERY_SPEED_MPS));

    Supplier<Translation2d> closestTrenchSupplier =() -> (
        drivetrain.getEstimatedPosition().getY() >= FieldConstants.fieldWidth / 2.0)? 
        FieldConstants.trenchRight : FieldConstants.trenchLeft;

    Supplier<Rotation2d> deliveryAngleSupplier = () -> 
    closestTrenchSupplier.get().minus(drivetrain.getEstimatedPosition().getTranslation()).getAngle();

    deliveryGroup.addCommands(new RunCommand(()-> shooter.keepVelocity(ShooterConstants.DELIVERY_SPEED_MPS)).alongWith(
            new DriveAndHomeToSupplierCommand(drivetrain, controller, deliveryAngleSupplier)
    ));

    return deliveryGroup;

}

}
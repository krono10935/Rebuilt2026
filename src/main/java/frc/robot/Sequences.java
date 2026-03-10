package frc.robot;

import java.util.Map;
import java.util.Set;
import java.util.TreeMap;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.FieldConstants.TowerSide;
import frc.robot.commands.Drivetrain.DriveAndHomeToSupplierCommand;
import frc.robot.commands.Drivetrain.DriveCommand;
import frc.robot.commands.SpinUpForDelivery;
import frc.robot.commands.IntakeCommands.CloseCommand;
import frc.robot.commands.IntakeCommands.IntakeCommand;
import frc.robot.commands.IntakeCommands.OpenCommand;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Shooter.IO.ShootRealConstants;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.Vision.ObjectDetection.ObjectDetection;
import frc.robot.subsystems.Vision.VisionConstants.CamerasConstants;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.utils.ErrorMessage;
import frc.utils.ParallelRaceGroupWithWinner;

import static frc.robot.FieldConstants.getTowerSideTargetPose;


public class Sequences {


    /**
     * Determines whether the robot is close enough to the tower
     * to safely open the climb.
     *
     * @param robotPose current robot pose
     * @return true if close enough to open climb
     */
    private static boolean closeEnoughToOpenClimb(Pose2d robotPose, Translation2d towerSidePose) {

        return robotPose.getTranslation().getDistance(towerSidePose)
                < ClimbConstants.MIN_DISTANCE_FROM_TOWER_TO_OPEN_CLIMB;
    }


    /**
     * Determines whether the robot is far enough from the tower
     * to safely close the climb.
     *
     * @param robotPose current robot pose
     * @return true if far enough to close climb
     */
    private static boolean farEnoughToCloseClimb(Pose2d robotPose, Translation2d towerSidePose) {

        return robotPose.getTranslation().getDistance(towerSidePose)
                >= ClimbConstants.MIN_DISTANCE_FROM_TOWER_TO_CLOSE_CLIMB;
    }

    private static boolean isComingFromTop(Pose2d robotPose){
        return robotPose.getX() > 4.0;
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

                climb.openCommand(),

                climb.closeCommand()
        );


        SequentialCommandGroup closeAndRetryIfFailed =
                new SequentialCommandGroup(

                        climb.closeCommand(),

                        retry.unless(climb::isAtSetPoint)
                );

        return closeAndRetryIfFailed;
    }


    /**
     * Closes and stops all relevant subsystems in parallel,
     * then performs an error check.
     *
     * @param intake  intake subsystem
     * @param climb   optional climb subsystem to close
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

                        new InstantCommand(shooter::stopFlyWheel, shooter),

                        new InstantCommand(() ->
                                shooter.toggleKicker(false))

                );

        if (climb != null){
                closeSubsystemsParallel.addCommands(climb.closeCommand());
        }

        return closeSubsystemsParallel;
    }


    /**
     * Closes all subsystems and then opens the climb.
     *
     * @param intake     intake subsystem
     * @param climb      climb subsystem
     * @param shooter    shooter subsystem
     * @return command that prepares and opens the climb
     */
    public static Command climbOpen(
            Intake intake,
            Climb climb,
            Shooter shooter
    ) {

        return new SequentialCommandGroup(
                closeSubsystems(intake, null, shooter), // Don't close climb
                climb.openCommand()
        );
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
            Supplier<TowerSide> climbSideSupplier,
            Shooter shooter,
            Vision vision
    ) {

        ParallelCommandGroup prepareForClimb = new ParallelCommandGroup(
                        closeSubsystems(intake, climb, shooter),

                        new StartEndCommand(() -> {
                                if(isComingFromTop(drivetrain.getEstimatedPosition())){
                                        vision.setCamAsPriority(CamerasConstants.SHOOTER_CAMERA);
                                }
                                else{
                                        // vision.setCamAsPriority(CamerasConstants.SIDE_CAMERA);
                                }
                                
                                },

                                () -> vision.clearPriority()));

        Command autoClimbSubsystemsSequence = prepareForClimb.withDeadline(new WaitUntilCommand(() ->
                                closeEnoughToOpenClimb(drivetrain.getEstimatedPosition(), 
                                getTowerSideTargetPose(climbSideSupplier.get(), false).getTranslation())))
                                .andThen(climb.openCommand());

        ParallelCommandGroup autoClimbSequence = autoClimbSubsystemsSequence
                .alongWith(new SelectCommand<TowerSide>(getClimbSideMap(drivetrain, false), climbSideSupplier));

                
        return autoClimbSequence.onlyIf(() ->
                DriverStation.getMatchTime() < 20
                        || DriverStation.isAutonomous()
        );
    }

    private static Map<TowerSide, Command> getClimbSideMap(Drivetrain drivetrain, boolean driveBack){
        Map<TowerSide, Command> commandMap = new TreeMap<TowerSide, Command>();
        for (TowerSide value : TowerSide.values()) {
                commandMap.put(value, drivetrain.driveToPose(getTowerSideTargetPose(value, driveBack)));
        }
        return commandMap;
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
            Supplier<TowerSide> climbSideSupplier,
            Shooter shooter,
            Vision vision
    ) {

        SequentialCommandGroup mainDeclimbCommand =
                new SequentialCommandGroup(climb.openCommand().alongWith(
                        new InstantCommand(() -> vision.clearPriority()),
                        
                        new SelectCommand<TowerSide>(
                                getClimbSideMap(drivetrain, true), climbSideSupplier),
                        
                        new WaitUntilCommand(() -> farEnoughToCloseClimb(
                        drivetrain.getEstimatedPosition(), getTowerSideTargetPose(
                                climbSideSupplier.get(), true).getTranslation()
                        )).andThen(
                                closeAndRetryClosingIfFailed(climb)
                        )
                )
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
        andThen(new IntakeCommand(intake));
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

        return CloseCommand.closeWithErrorHandeling(intake)
        .beforeStarting(new InstantCommand(() -> intake.setPercent(0)));
    }

    private static boolean matchesDeliveryChassisSpeeds(ChassisSpeeds currentSpeeds){
        return  Math.abs(ShooterConstants.DELIVERY_CHASSIS_SPEEDS.vxMetersPerSecond - currentSpeeds.vxMetersPerSecond) 
                       < ShooterConstants.XY_DELIVERY_SPEED_TOLERANCE && 
                Math.abs(ShooterConstants.DELIVERY_CHASSIS_SPEEDS.vyMetersPerSecond - currentSpeeds.vyMetersPerSecond) 
                       < ShooterConstants.XY_DELIVERY_SPEED_TOLERANCE &&
                Math.abs(ShooterConstants.DELIVERY_CHASSIS_SPEEDS.omegaRadiansPerSecond - currentSpeeds.omegaRadiansPerSecond) 
                       < ShooterConstants.OMEGA_DELIVERY_SPEED_TOLERANCE_RADIANS;
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
        Alert hoodAimingFailed = new Alert("Hood failed to aim", AlertType.kError);
        Alert shooterFailedToKeepVelocity = new Alert("Shooter failed to keep velocity!", AlertType.kError);

        SequentialCommandGroup spinUpAndAimHood =
                new SequentialCommandGroup();

        spinUpAndAimHood.addCommands(new DriveCommand(drivetrain, controller).withDeadline(
                new SpinUpForDelivery(
                        drivetrain,
                        shooter,
                        ShooterConstants.DELIVERY_SPEED_MPS
                )));

        Supplier<Translation2d> closestTrenchSupplier =
                () -> (drivetrain.getEstimatedPosition().getY()
                        >= FieldConstants.fieldWidth / 2.0)
                        ? FieldConstants.RightTrench.openingTopCenter.toTranslation2d()
                        : FieldConstants.LeftTrench.openingTopCenter.toTranslation2d();


        Supplier<Rotation2d> deliveryAngleSupplier =
                () -> closestTrenchSupplier.get()
                        .minus(
                                drivetrain.getEstimatedPosition()
                                        .getTranslation()
                        )
                        .getAngle();



        ParallelCommandGroup deliver = new ParallelCommandGroup(
                (new RunCommand(() ->
                        shooter.keepVelocity(ShooterConstants.DELIVERY_SPEED_MPS)
                        ).raceWith(ParallelRaceGroupWithWinner.andThenOnlyIfTimeout(
                                new WaitUntilCommand(() -> shooter.isShooterAtGoal())
                                .andThen(new InstantCommand(() -> shooterFailedToKeepVelocity.set(false))),

                                ShootRealConstants.FLYWHEEL_TIME_TO_REACH_GOAL,

                                new InstantCommand(shooter::stopFlyWheel)
                        .andThen(new InstantCommand(() -> shooterFailedToKeepVelocity.set(true))))
                        )).onlyIf(
                                () -> ObjectDetection.getInstance().hasBalls() &&
                                 matchesDeliveryChassisSpeeds(drivetrain.getChassisSpeeds())),


                new DriveAndHomeToSupplierCommand(
                        drivetrain,
                        controller,
                        deliveryAngleSupplier
                )
                

        );

        return spinUpAndAimHood.andThen(deliver);
    }

}

package frc.robot;

import java.util.Map;
import java.util.TreeMap;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.FieldConstants.TowerSide;
import frc.robot.commands.Drivetrain.*;
import frc.robot.commands.Shooter.ShootForDelivery;
import frc.robot.commands.SpinUpForDelivery;
import frc.robot.commands.IntakeCommands.*;
import frc.robot.subsystems.Shooter.*;
import frc.robot.subsystems.Shooter.IO.ShootRealConstants;
import frc.robot.subsystems.Vision.*;
import frc.robot.subsystems.Vision.ObjectDetection.ObjectDetection;
import frc.robot.subsystems.Vision.VisionConstants.CamerasConstants;
import frc.robot.subsystems.climb.*;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.utils.ParallelRaceGroupWithWinner;

import static frc.robot.FieldConstants.getTowerSideTargetPose;


public class Sequences {

    /**
     * Checks if robot is close enough to the tower to safely open climb.
     * @param robotPose the robot pose
     * @param towerSidePose the pose of the side of the tower that you choose
     */
    private static boolean closeEnoughToOpenClimb(Pose2d robotPose, Translation2d towerSidePose) {
        return robotPose.getTranslation().getDistance(towerSidePose)
                < ClimbConstants.MIN_DISTANCE_FROM_TOWER_TO_OPEN_CLIMB;
    }

    /**
     * Checks if robot is far enoughto the tower to safely close climb.
     * @param robotPose the robot pose
     * @param towerSidePose the pose of the side of the tower that you choose
     */
    private static boolean farEnoughToCloseClimb(Pose2d robotPose, Translation2d towerSidePose) {
        return robotPose.getTranslation().getDistance(towerSidePose)
                >= ClimbConstants.MIN_DISTANCE_FROM_TOWER_TO_CLOSE_CLIMB;
    }

    /**
     * Determines from which direction (top or bottom of the field)
     * the robot is approaching the tower.
     * @param robotPose the robot pose
     */
    private static boolean isComingFromTop(Pose2d robotPose) {
        return robotPose.getX() > 4.0; //TODO Figure out logic of sides, TOP != the same side in both Alliance zone ALSO NO MAGIC NUMBER MAKE IT CONSTANT
    }

    /**
     * Attempts to close climb, retries once if failed.
     * @param climb
     */
    private static Command closeAndRetryClosingIfFailed(Climb climb) {

        SequentialCommandGroup retry = new SequentialCommandGroup(
                climb.openCommand(),
                climb.closeCommand()
        );

        return new SequentialCommandGroup(
                climb.closeCommand(),
                retry.unless(climb::isAtSetPoint)
        );
    }

    /**
     * A command which stops intake, shooter, and optionally climb (if climb isn't null).
     * @param intake
     * @param climb
     * @param shooter
     * @return the command
     */
    private static Command closeSubsystems(Intake intake, Climb climb, Shooter shooter) {

        ParallelCommandGroup group = new ParallelCommandGroup(
                stopIntakeAndClose(intake),
                shooter.getIndexer().turnOffIndexerCommand(),
                new InstantCommand(shooter::stopFlyWheel, shooter),
                new InstantCommand(() -> shooter.toggleKicker(false))
        );

        if (climb != null) {
            group.addCommands(climb.closeCommand());
        }

        return group;
    }

    /**
     * Closes everything and opens climb.
     * @param intake
     * @param climb
     * @param shooter
     */
    public static Command climbOpen(Intake intake, Climb climb, Shooter shooter) {
        return new SequentialCommandGroup(
                closeSubsystems(intake, null, shooter),
                climb.openCommand()
        );
    }

    /**
     * Full autonomous climb routine.
     * @param intake
     * @param drivetrain
     * @param climb
     * @param climbSideSupplier which side to climb to
     * @param shooter
     * @param vision
     * @return A full climb command
     */
    public static Command autoClimb(
            Intake intake,
            Drivetrain drivetrain,
            Climb climb,
            Supplier<TowerSide> climbSideSupplier,
            Shooter shooter,
            Vision vision
    ) {

        ParallelCommandGroup prepare = new ParallelCommandGroup(
                closeSubsystems(intake, climb, shooter),

                new StartEndCommand(
                        () -> {
                            if (isComingFromTop(drivetrain.getEstimatedPosition())) {
                                vision.setCamAsPriority(CamerasConstants.SHOOTER_CAMERA);
                            }
                        },
                        vision::clearPriority
                )
        );

        Command sequence =
                prepare.withDeadline(new WaitUntilCommand(() ->
                        closeEnoughToOpenClimb(
                                drivetrain.getEstimatedPosition(),
                                getTowerSideTargetPose(climbSideSupplier.get(), false).getTranslation()
                        )
                )).andThen(climb.openCommand());

        return sequence
                .alongWith(new SelectCommand<>(getClimbSideMap(drivetrain, false), climbSideSupplier))
                .onlyIf(() ->
                        DriverStation.getMatchTime() < 20 || DriverStation.isAutonomous()
                );
    }

    /**
     * Builds map for climb side driving.
     * @param drivetrain
     * @param driveBack Whether or not to drive backwards when the declimb is finished
     * @return
     */
    private static Map<TowerSide, Command> getClimbSideMap(Drivetrain drivetrain, boolean driveBack) {
        Map<TowerSide, Command> map = new TreeMap<>();

        for (TowerSide value : TowerSide.values()) {
            map.put(value, drivetrain.driveToPose(getTowerSideTargetPose(value, driveBack)));
        }

        return map;
    }

    /**
     * Autonomous declimb routine.
     * @param intake
     * @param drivetrain
     * @param climb
     * @param climbSideSupplier Which side to climb to
     * @param shooter
     * @param vision
     * @return the command for the auto declimb
     */
    public static Command autoDeclimb(
            Intake intake,
            Drivetrain drivetrain,
            Climb climb,
            Supplier<TowerSide> climbSideSupplier,
            Shooter shooter,
            Vision vision
    ) {

        SequentialCommandGroup main = new SequentialCommandGroup(
                climb.openCommand().alongWith( //TODO fix alongWith the robot should not descend while the swerve is spinning could cause wrong position
                        new InstantCommand(vision::clearPriority),

                        new SelectCommand<>(getClimbSideMap(drivetrain, true), climbSideSupplier),

                        new WaitUntilCommand(() ->
                                farEnoughToCloseClimb(
                                        drivetrain.getEstimatedPosition(),
                                        getTowerSideTargetPose(climbSideSupplier.get(), true).getTranslation()
                                )
                        ).andThen(closeAndRetryClosingIfFailed(climb))
                )
        );

        return main.onlyIf(climb::getHasClimbed);
    }

    /**
     * Opens intake safely and starts motor.
     * @param intake
     */
    public static Command intakeOpenStart(Intake intake) {
        return OpenCommand.openWithErrorHandeling(intake)
                .andThen(new IntakeCommand(intake));
    }

    /**
     * Stops intake and safely closes it.
     * @param intake
     */
    public static Command stopIntakeAndClose(Intake intake) {
        return CloseCommand.closeWithErrorHandeling(intake)
                .beforeStarting(new InstantCommand(() -> intake.setPercent(0)));
    }

    /**
     * Checks if chassis speeds match delivery target.
     * @param speeds The current ChassisSpeeds
     */
    private static boolean matchesDeliveryChassisSpeeds(ChassisSpeeds speeds) {

        return Math.abs(ShooterConstants.DELIVERY_CHASSIS_SPEEDS.vxMetersPerSecond - speeds.vxMetersPerSecond)
                < ShooterConstants.XY_DELIVERY_SPEED_TOLERANCE

                && Math.abs(ShooterConstants.DELIVERY_CHASSIS_SPEEDS.vyMetersPerSecond - speeds.vyMetersPerSecond)
                < ShooterConstants.XY_DELIVERY_SPEED_TOLERANCE

                && Math.abs(ShooterConstants.DELIVERY_CHASSIS_SPEEDS.omegaRadiansPerSecond - speeds.omegaRadiansPerSecond)
                < ShooterConstants.OMEGA_DELIVERY_SPEED_TOLERANCE_RADIANS;
    }

    /**
     * Delivery sequence:
     * spins shooter, aligns robot, and fires.
     * @param drivetrain
     * @param shooter
     * @param controller the driver's controller
     */
    public static Command delivery(
            Drivetrain drivetrain,
            Shooter shooter,
            CommandXboxController controller
    ) {


        BooleanSupplier isRobotStopped = () ->
                drivetrain.getChassisSpeeds().vxMetersPerSecond < 0.1 &&
                        drivetrain.getChassisSpeeds().vyMetersPerSecond < 0.1 &&
                        drivetrain.getChassisSpeeds().omegaRadiansPerSecond < 0.1;



        Supplier<Translation2d> trench = () -> FieldConstants.getClosestTrench(drivetrain.getEstimatedPosition());


        BooleanSupplier isRobotAligned = () -> trench.get().
                minus(drivetrain.getEstimatedPositionFlipped().getTranslation()).getAngle().getDegrees() < 5;

        Supplier<Rotation2d> angle = () -> trench.get()
                        .minus(drivetrain.getEstimatedPositionFlipped().getTranslation())
                        .getAngle();

        Command shooting = new ConditionalCommand(new ShootForDelivery(shooter), new InstantCommand(() -> {}), isRobotAligned);
        Command homing = new ConditionalCommand(new DriveAndHomeToSupplierCommand(drivetrain, controller, angle),
                new InstantCommand(() -> {}), isRobotStopped);


        return shooting.alongWith(homing);
    }
}
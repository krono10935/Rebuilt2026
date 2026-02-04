package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
                                Drivetrain drivetrain , CommandXboxController  CommandXboxController) {
        SequentialCommandGroup shooterCommand = new SequentialCommandGroup();
        shooterCommand.addCommands(indexer.turnOnIndexerCommand());
        shooterCommand.addCommands
                (new ShootCommand(shooter,drivetrain,drivetrain::getEstimatedPosition, (pose) -> true).
                        alongWith(
                                new DriveAndHomeCommand(drivetrain,
                                        CommandXboxController,()-> shooter.getShootParameters().robotAngle())));
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
        deliveryCommand.addCommands(Sequences.openIntakeStart(intake));
        deliveryCommand.addCommands(new InstantCommand(()-> shooter.setHoodAngle(Rotation2d.fromDegrees(45))));
        deliveryCommand.addCommands(indexer.turnOnIndexerCommand());
        deliveryCommand.addCommands(new ShootCommand(shooter,drivetrain,drivetrain::getEstimatedPosition,(pose) -> true));
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
                Sequences.closeIntakeStop(intake),
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
     * Opens and starts the intake
     * @param intake
     * @return
     */
    private static Command openIntakeStart(Intake intake){
        SequentialCommandGroup intakeCommand = new SequentialCommandGroup();
        intakeCommand.addCommands(new OpenCommand(intake));
        intakeCommand.addCommands(new IntakeCommand(intake));
        return intakeCommand;
    }

    /**
     * Closes and stops the intake
     * @param intake
     * @return
     */
    private static Command closeIntakeStop(Intake intake){
        SequentialCommandGroup intakeCommand = new SequentialCommandGroup();
        intakeCommand.addCommands(new InstantCommand(intake::stopIntakeMotor));
        intakeCommand.addCommands(new CloseCommand(intake));
        return intakeCommand;
    }
}
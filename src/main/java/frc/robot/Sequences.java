package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.IntakeCommands.CloseCommand;
import frc.robot.commands.IntakeCommands.IntakeCommand;
import frc.robot.commands.IntakeCommands.OpenCommand;
import frc.robot.commands.Shooter.HomeHoodCommand;
import frc.robot.commands.Shooter.ShootCommand;
import frc.robot.commands.Shooter.SpinUp;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;

public class Sequences {
    public static Command shoot(Shooter shooter, Indexer indexer,Drivetrain drivetrain ) {
        SequentialCommandGroup shooterCommand = new SequentialCommandGroup();
        shooterCommand.addCommands(new HomeHoodCommand(shooter, drivetrain::getEstimatedPosition,(pose) -> Rotation2d.kZero));
        shooterCommand.addCommands(indexer.turnOnIndexerCommand());
        shooterCommand.addCommands(new ShootCommand(shooter,drivetrain::getEstimatedPosition,(pose) -> true));
        return shooterCommand;
    }

    public static Command delivery(Shooter shooter, Indexer indexer, Intake intake, Drivetrain drivetrain) {
        SequentialCommandGroup deliveryCommand = new SequentialCommandGroup();
        deliveryCommand.addCommands(Sequences.openIntakeStart(intake));
        deliveryCommand.addCommands(new InstantCommand(()-> shooter.setHoodAngle(Rotation2d.fromDegrees(45))));
        deliveryCommand.addCommands(indexer.turnOnIndexerCommand());
        deliveryCommand.addCommands(new ShootCommand(shooter,drivetrain::getEstimatedPosition,(pose) -> true));
        return deliveryCommand;
    }

    public static Command Climb(Intake intake, Climb climb, Indexer indexer,Shooter shooter, Drivetrain drivetrain) {
        SequentialCommandGroup climbCommand = new SequentialCommandGroup();
        if(climb.getClimbState() == ClimbConstants.ClimbState.CLOSED) {
            ParallelCommandGroup closeSubsystems = new ParallelCommandGroup();
            closeSubsystems.addCommands(Sequences.closeIntakeStop(intake));
            closeSubsystems.addCommands(indexer.turnOffIndexerCommand());
            closeSubsystems.addCommands(new ShootCommand(shooter,drivetrain::getEstimatedPosition,(pose) -> false));
            climbCommand.addCommands(closeSubsystems);
            //climbCommand.addCommands(drivetrain.driveToPose(new Pose2d()));
            //TODO add drive to pose command
            climbCommand.addCommands(climb.openCommand());
        }else if(climb.getClimbState() == ClimbConstants.ClimbState.OPEN) {
            climbCommand.addCommands(climb.closeCommand());
        }
        return climbCommand;
    }

    public static Command toggleIntake(Intake intake) {
        return intake.getIsIntakeOpen() ? closeIntakeStop(intake) : openIntakeStart(intake);
    }

    public static Command openIntakeStart(Intake intake){
        SequentialCommandGroup intakeCommand = new SequentialCommandGroup();
        intakeCommand.addCommands(intake.openIntake());
        intakeCommand.addCommands(intake.startIntake());
        return intakeCommand;
    }

    public static Command closeIntakeStop(Intake intake){
        SequentialCommandGroup intakeCommand = new SequentialCommandGroup();
        intakeCommand.addCommands(intake.stopIntake());
        intakeCommand.addCommands(intake.closeIntake());
        return intakeCommand;
    }
}

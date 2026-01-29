package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
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
        shooterCommand.addCommands(new SpinUp(shooter).alongWith(indexer.turnOnIndexerCommand()));
        shooterCommand.addCommands(new ShootCommand(shooter,drivetrain::getEstimatedPosition,(pose) -> true));
        return shooterCommand;
    }

    public static Command delivery(Shooter shooter, Indexer indexer, Intake intake, Drivetrain drivetrain,IntakeConstants intakeConstants ) {
        SequentialCommandGroup deliveryCommand = new SequentialCommandGroup();
        deliveryCommand.addCommands(Sequences.openIntakeStart(intake,intakeConstants));
        deliveryCommand.addCommands(new HomeHoodCommand(shooter, drivetrain::getEstimatedPosition,(pose) -> Rotation2d.kZero));
        deliveryCommand.addCommands(Sequences.shoot(shooter,indexer,drivetrain));
        return deliveryCommand;
    }

    public static Command Climb(Intake intake, IntakeConstants intakeConstands, Climb climb,Indexer indexer){
        SequentialCommandGroup climbCommand = new SequentialCommandGroup();
        if(climb.getClimbState() == ClimbConstants.ClimbState.CLOSED) {
            climbCommand.addCommands(Sequences.closeIntakeStop(intake,intakeConstands));
            climbCommand.addCommands(indexer.turnOffIndexerCommand());
            climbCommand.addCommands(climb.openCommand());
        }else if(climb.getClimbState() == ClimbConstants.ClimbState.OPEN) {
            climbCommand.addCommands(climb.closeCommand());
        }
        return climbCommand;
    }

    public static Command openIntakeStart(Intake intake, IntakeConstants intakeConstants){
        SequentialCommandGroup intakeCommand = new SequentialCommandGroup();
        intakeCommand.addCommands(new OpenCommand(intake,intakeConstants));
        intakeCommand.addCommands(new IntakeCommand(intake));
        return intakeCommand;
    }

    public static Command closeIntakeStop(Intake intake, IntakeConstants intakeConstands){
        SequentialCommandGroup intakeCommand = new SequentialCommandGroup();
        intakeCommand.addCommands(new CloseCommand(intake,intakeConstands));
        intakeCommand.addCommands(new IntakeCommand(intake));
        return intakeCommand;
    }
}

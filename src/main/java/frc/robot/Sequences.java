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
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;

public class Sequences {
    //TODO fix the sequences
    public static Command shoot(Shooter shooter, Indexer indexer) {
        SequentialCommandGroup shooterCommand = new SequentialCommandGroup();
        //shooterCommand.addCommands(new HomeHoodCommand())
        shooterCommand.addCommands(new SpinUp(shooter).alongWith(indexer.turnOnIndexerCommand()));
        //shooterCommand.addCommandshootCommand().alongWith);
        return shooterCommand;
    }

    public static Command delivery(Shooter shooter, Indexer indexer, Intake intake) {
        SequentialCommandGroup deliveryCommand = new SequentialCommandGroup();
        deliveryCommand.addCommands(new IntakeCommand(intake));
        //deliveryCommand.addCommands(new HomeHoodCommand()); set hood degree
        deliveryCommand.addCommands(Sequences.shoot(shooter, indexer));
        return deliveryCommand;
    }

    public static Command Climb(Intake intake, IntakeConstants intakeConstands, Climb climb,Indexer indexer){
        SequentialCommandGroup climbCommand = new SequentialCommandGroup();
        climbCommand.addCommands(new CloseCommand(intake,intakeConstands));
        climbCommand.addCommands(indexer.turnOffIndexerCommand());
        if(climb.getClimbState() == ClimbConstants.ClimbState.CLOSED) {

            climbCommand.addCommands(climb.openCommand());
        }else if(climb.getClimbState() == ClimbConstants.ClimbState.OPEN) {
            climbCommand.addCommands(climb.closeCommand());
        }
        return climbCommand;
    }

    public static Command openIntakeStart(Intake intake, IntakeConstants intakeConstands){
        SequentialCommandGroup intakeCommand = new SequentialCommandGroup();
        intakeCommand.addCommands(new OpenCommand(intake,intakeConstands));
        intakeCommand.addCommands(new IntakeCommand(intake));
        return intakeCommand;
    }

    public static Command closeIntakeStop(Intake intake, IntakeConstants intakeConstands){
        SequentialCommandGroup intakeCommand = new SequentialCommandGroup();
        intakeCommand.addCommands(new CloseCommand(intake,intakeConstands));
        // TODO change to stop intake
        intakeCommand.addCommands(new IntakeCommand(intake));
        return intakeCommand;
    }
}

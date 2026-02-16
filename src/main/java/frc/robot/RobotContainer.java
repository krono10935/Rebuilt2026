// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveAndHomeCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.DriveRobotRelative;
import frc.robot.commands.IntakeCommands.CloseCommand;
import frc.robot.commands.IntakeCommands.IntakeCommand;
import frc.robot.commands.IntakeCommands.OpenCommand;
import frc.robot.commands.Shooter.ShootCommand;
import frc.robot.commands.Shooter.SpinUp;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.climb.Climb;
import frc.robot.leds.LedManager;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.intake.Intake;

import frc.robot.subsystems.drivetrain.PPController;
import org.littletonrobotics.conduit.ConduitApi;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class RobotContainer
{   
    private static RobotContainer instance;

    public final LedManager ledManager;

    public final Vision vision;

    public final Shooter shooter;

    public final Intake intake;

    public final Climb climb;

    private final CommandXboxController xboxController;

    public final Drivetrain drivetrain;

    private final LoggedDashboardChooser<Command> chooser; 

    private final LoggedDashboardChooser<Boolean> isClimbAuto;


    public static RobotContainer getInstance(){
        if (instance == null){
            instance = new RobotContainer();
        }
        return instance;
    }

    private RobotContainer()
    {
        isClimbAuto = new LoggedDashboardChooser<>("is climb automatic");
        isClimbAuto.addDefaultOption("yes", true);
        isClimbAuto.addOption("no", false);

        shooter = new Shooter();

        intake = new Intake();

        climb = new Climb();

        xboxController = new CommandXboxController(0);

        drivetrain = new Drivetrain(ConduitApi.getInstance()::getPDPVoltage, Constants.CHASSIS_TYPE.constants);

        vision = new Vision(drivetrain::addVisionMeasurement, drivetrain::getEstimatedPosition);

        chooser = new LoggedDashboardChooser<>("chooser", AutoBuilder.buildAutoChooser());

        configureBindings();
        ledManager = new LedManager();
    }

    private void configurePitBindings() {
        xboxController.a().onTrue(new SpinUp(shooter)
        .until(shooter::isShooterAtGoal)
        .andThen(new RunCommand(() -> shooter.keepVelocity(ShooterConstants.SHOOTING_SPEED), shooter)));

        // Disable all subsystems commands
        xboxController.b().onTrue(new InstantCommand(() -> {
            drivetrain.getCurrentCommand().cancel();
            CommandScheduler.getInstance().schedule(drivetrain.idle());
            shooter.getCurrentCommand().cancel();
            CommandScheduler.getInstance().schedule(shooter.idle());
            intake.getCurrentCommand().cancel();
            CommandScheduler.getInstance().schedule(intake.idle());
            climb.getCurrentCommand().cancel();
            CommandScheduler.getInstance().schedule(climb.idle());
        }));

        xboxController.y().debounce(0.3).whileTrue(new OpenCommand(intake));

        xboxController.y().onFalse(new CloseCommand(intake));

        xboxController.x().onTrue(new IntakeCommand(intake).onlyIf(intake::isOpen));

        xboxController.povDown().onTrue(climb.closeCommand());

        xboxController.povUp().onTrue(climb.openCommand());

        xboxController.povCenter().onTrue(drivetrain.resetGyro());
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(new DriveCommand(drivetrain, xboxController));

        BooleanSupplier isHubActive = () -> {
            double time = DriverStation.getMatchTime();

            return Constants.HubTiming.isActive(time) ||
                    Constants.HubTiming.isActive(time - Constants.HUB_ACTIVITY_DEABAND_BEFORE_ACTIVE) ||
                    Constants.HubTiming.isActive(time + Constants.HUB_ACTIVITY_DEABAND_AFTER_ACTIVE);
        };
       new Trigger(isHubActive).
               and(() -> FieldConstants.isInAllianceZone(drivetrain.getEstimatedPosition()))
        .whileTrue(ShootCommand.shootCommandFactory(shooter,drivetrain,xboxController,intake, vision));

        xboxController.y().toggleOnTrue(Sequences.intakeOpenStart(intake).alongWith(new DriveRobotRelative(drivetrain, xboxController)));

        xboxController.povUp().whileTrue(Sequences.autoClimb(intake, drivetrain, climb, shooter));

        xboxController.b().whileTrue(climb.closeCommand());

        xboxController.x().onTrue(
            Sequences.delivery(drivetrain, shooter, xboxController,intake));

        xboxController.a().onTrue(drivetrain.resetGyro());
        
    }
    public Command getAutonomousCommand()
    {
        return chooser.get();
    }

    public void registerNamedCommand(DriveAndHomeCommand driveAndHomeCommand){

        Command aimRobot = new StartEndCommand(() -> {
            PPController.setThetaOverride(driveAndHomeCommand::calculateThetaPID);
        }, PPController::clearThetaOverride);

        Command aimRobotStationary = new RunCommand(
                () -> drivetrain.drive(new ChassisSpeeds(
                        0, 0, driveAndHomeCommand.calculateThetaPID())), drivetrain);


        NamedCommands.registerCommand("shootAndAimMoving",
                ShootCommand.shootCommandFactory(shooter, drivetrain, xboxController,intake, vision).beforeStarting(new SpinUp(shooter))
                        .alongWith(aimRobot));

        NamedCommands.registerCommand("shootAndAimStationary",
                ShootCommand.shootCommandFactory(shooter, drivetrain, xboxController,intake, vision).beforeStarting(new SpinUp(shooter))
                        .alongWith(aimRobotStationary));

        NamedCommands.registerCommand("spinUp", new SpinUp(shooter));

        NamedCommands.registerCommand("waitUntilNoBalls", new WaitUntilCommand(() ->
                !new Intake().hasBalls()));

        NamedCommands.registerCommand("openIntake",
                new SequentialCommandGroup(Sequences.intakeOpenStart(intake)));
        NamedCommands.registerCommand("closeIntake",
                new SequentialCommandGroup(Sequences.stopIntakeAndClose(intake)));

        NamedCommands.registerCommand("openClimb", new Climb().openCommand());
        NamedCommands.registerCommand("closeClimb", new Climb().closeCommand());
    }


}
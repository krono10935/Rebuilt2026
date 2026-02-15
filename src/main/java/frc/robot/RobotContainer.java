// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveAndHomeCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.IntakeCommands.CloseCommand;
import frc.robot.commands.IntakeCommands.IntakeCommand;
import frc.robot.commands.IntakeCommands.OpenCommand;
import frc.robot.commands.Shooter.ShootCommand;
import frc.robot.commands.Shooter.SpinUp;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.climb.Climb;
import frc.robot.leds.LedLocation;
import frc.robot.leds.LedManager;
import frc.robot.leds.LedPattern;
import frc.robot.leds.LedState;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.intake.Intake;

import frc.robot.subsystems.drivetrain.PPController;
import org.littletonrobotics.conduit.ConduitApi;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;

public class RobotContainer
{   
    private static RobotContainer instance;

    public final LedManager ledManager;

    private final Vision vision;

    public final Shooter shooter;

    public final Intake intake;

    public final Climb climb;

    private final CommandXboxController xboxController;

    public final Drivetrain drivetrain;

    private final LoggedDashboardChooser<Command> chooser;

    private final LoggedNetworkNumber hoodAngle;


    public static RobotContainer getInstance(){
        if (instance == null){
            instance = new RobotContainer();
        }
        return instance;
    }

    private RobotContainer()
    {
        hoodAngle = new LoggedNetworkNumber("HoodAngle", 0);
        shooter = new Shooter();

        intake = new Intake();

        climb = new Climb();

        xboxController = new CommandXboxController(0);

        drivetrain = new Drivetrain(ConduitApi.getInstance()::getPDPVoltage, Constants.CHASSIS_TYPE.constants);

        vision = new Vision(drivetrain::addVisionMeasurement, drivetrain::getEstimatedPosition);

        chooser = new LoggedDashboardChooser<>("chooser", AutoBuilder.buildAutoChooser());

        chooser.addOption("shit", drivetrain.driveToPose(new Pose2d(3, 5, Rotation2d.kZero)));

        configureBindings();
        ledManager = new LedManager();
        ledManager.setColors(new LedState(LedPattern.BRWON, Color.kDarkBlue, Color.kCyan, 0.25, 0.7, LedLocation.BASE));
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
        xboxController.y().onTrue(Sequences.intakeOpenStart(intake)); 
        xboxController.y().onFalse(new InstantCommand(intake::stopIntakeMotor));

        drivetrain.setDefaultCommand(
            new ConditionalCommand(
                ShootCommand.shootCommandFactory(shooter, drivetrain, xboxController),
                 new DriveCommand(drivetrain, xboxController), () -> Constants.HubTiming.isActive(DriverStation.getMatchTime())));

        xboxController.leftBumper().onTrue(Sequences.autoClimb(intake, drivetrain, climb, shooter));
        xboxController.leftBumper().debounce(0.3).onTrue(Sequences.climbOpen(intake, drivetrain, climb, shooter));

        xboxController.rightBumper().onTrue(Sequences.climbClose(intake, climb, shooter, drivetrain));

        xboxController.x().onTrue(
            Sequences.delivery(drivetrain, shooter, xboxController));

        xboxController.a().onTrue(new DriveCommand(drivetrain, xboxController)); 

        xboxController.b().onTrue(
            new InstantCommand(() -> drivetrain.getCurrentCommand().cancel())
            .onlyIf(()->drivetrain.getCurrentCommand() != null));

        xboxController.povUp().onTrue(drivetrain.resetGyro());
        
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
                ShootCommand.shootCommandFactory(shooter, drivetrain, xboxController).beforeStarting(new SpinUp(shooter))
                        .alongWith(aimRobot));

        NamedCommands.registerCommand("shootAndAimStationary",
                ShootCommand.shootCommandFactory(shooter, drivetrain, xboxController).beforeStarting(new SpinUp(shooter))
                        .alongWith(aimRobotStationary));

        NamedCommands.registerCommand("spinUp", new SpinUp(shooter));

        NamedCommands.registerCommand("waitUntilNoBalls", new WaitUntilCommand(() ->
                new Intake().getBalls()== 0));

        NamedCommands.registerCommand("openIntake",
                new SequentialCommandGroup(Sequences.intakeOpenStart(intake)));
        NamedCommands.registerCommand("closeIntake",
                new SequentialCommandGroup(Sequences.stopIntakeAndClose(intake)));

        NamedCommands.registerCommand("openClimb", new Climb().openCommand());
        NamedCommands.registerCommand("closeClimb", new Climb().closeCommand());
    }


}
// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.Shooter.ShootCommand;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.leds.LedLocation;
import frc.robot.leds.LedManager;
import frc.robot.leds.LedPattern;
import frc.robot.leds.LedState;
import frc.robot.subsystems.drivetrain.Drivetrain;

import org.littletonrobotics.conduit.ConduitApi;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.util.Color;


public class RobotContainer
{   
    private static RobotContainer instance;

    public final LedManager ledManager;
    private final Vision vision;

    private final Shooter shooter;

    private final CommandXboxController xboxController;

    public final Drivetrain drivetrain;

    private final LoggedDashboardChooser<Command> chooser;


    public static RobotContainer getInstance(){
        if (instance == null){
            instance = new RobotContainer();
        }
        return instance;
    }

    private RobotContainer()
    {
        shooter = new Shooter();

        xboxController = new CommandXboxController(0);

        drivetrain = new Drivetrain(ConduitApi.getInstance()::getPDPVoltage, Constants.CHASSIS_TYPE.constants);

        vision = new Vision(drivetrain::addVisionMeasurement, drivetrain::getEstimatedPosition);

        chooser = new LoggedDashboardChooser<>("chooser", AutoBuilder.buildAutoChooser());

        chooser.addOption("shit", drivetrain.driveToPose(new Pose2d(3, 5, Rotation2d.kZero)));

        configureBindings();
        ledManager = new LedManager();
        ledManager.setColors(new LedState(LedPattern.BRWON, Color.kDarkBlue, Color.kCyan, 0.25, 0.7, LedLocation.BASE));
    }

    private void configureBindings() {
        // drivetrain.setDefaultCommand(new DriveCommand(drivetrain, xboxController));
        xboxController.a().onTrue(new InstantCommand(() -> shooter.spinUp(17)));
        xboxController.b().whileTrue(new InstantCommand(() -> shooter.keepVelocity(17)).repeatedly());
        xboxController.x().onTrue(new InstantCommand(() -> shooter.stopFlyWheel()));
        // shooter.setDefaultCommand(ShootCommand.shootCommandFactory(shooter, drivetrain, xboxController));
    }
    public Command getAutonomousCommand()
    {
        return chooser.get();
    }


}
// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.drivetrain.Drivetrain;

import org.littletonrobotics.conduit.ConduitApi;
import com.pathplanner.lib.auto.AutoBuilder;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;


public class RobotContainer
{

    private static RobotContainer instance;

    public final Drivetrain drivetrain;
    public final CommandXboxController Xbox;
    private final LoggedDashboardChooser<Command> chooser;



    public static RobotContainer getInstance(){
        if (instance == null){
            instance = new RobotContainer();
        }
        return instance;
    }

    private RobotContainer()
    {

        drivetrain = new Drivetrain(ConduitApi.getInstance()::getPDPVoltage, Constants.CHASSIS_TYPE.constants);
        Xbox = new CommandXboxController(0);




        configureBindings();
        chooser = new LoggedDashboardChooser<>("chooser", AutoBuilder.buildAutoChooser());

        drivetrain.setDefaultCommand(new DriveCommand(drivetrain, Xbox));

    }

    private void configureBindings() {
        Xbox.a().onTrue(new InstantCommand( () -> drivetrain.reset(new Pose2d(
            drivetrain.getEstimatedPosition().getTranslation(),  Rotation2d.kZero
        ))));

     }
    
    
    public Command getAutonomousCommand()
    {
        return chooser.get();
    }


}
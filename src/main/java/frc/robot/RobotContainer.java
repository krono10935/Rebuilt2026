// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.drivetrain.Drivetrain;
import org.littletonrobotics.conduit.ConduitApi;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.DriveToPose;
import com.pathplanner.lib.path.DriveToPoseConstants;
import edu.wpi.first.wpilibj2.command.*;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;


public class RobotContainer
{

    private static RobotContainer instance;

    private final Shooter shooter;

    // private final Drivetrain drivetrain;

    // private final LoggedDashboardChooser<Command> chooser;


    public static RobotContainer getInstance(){
        if (instance == null){
            instance = new RobotContainer();
        }
        return instance;
    }

    private RobotContainer()
    {

        shooter = new Shooter();

        // drivetrain = new Drivetrain(ConduitApi.getInstance()::getPDPVoltage, Constants.CHASSIS_TYPE.constants);

        configureBindings();
        // chooser = new LoggedDashboardChooser<>("chooser", AutoBuilder.buildAutoChooser());
    }

    private void configureBindings() {
        shooter.shoot();

    }
    
    
    public Command getAutonomousCommand()
    {
        return null;
        // return chooser.get();
    }


}
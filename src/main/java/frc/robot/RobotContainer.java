// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommand;
import frc.robot.leds.LedLocation;
import frc.robot.leds.LedManager;
import frc.robot.leds.LedPattern;
import frc.robot.leds.LedState;
import frc.robot.subsystems.drivetrain.Drivetrain;
import org.littletonrobotics.conduit.ConduitApi;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.DriveToPose;
import com.pathplanner.lib.path.DriveToPoseConstants;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj.util.Color;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;


public class RobotContainer
{

    private static RobotContainer instance;

    public final Drivetrain drivetrain;
    public final LedManager ledManager;

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

        configureBindings();
        chooser = new LoggedDashboardChooser<>("chooser", AutoBuilder.buildAutoChooser());


        ledManager = new LedManager();

        ledManager.setColors(new LedState(LedPattern.BRWON, Color.kDarkBlue, Color.kCyan, 0.25, 0.7, LedLocation.BASE));



        // var ledCommand = new WaitCommand(15).andThen(new InstantCommand(() -> ledManager.turnOffAllLED()), new WaitCommand(10), new InstantCommand(() -> ledManager.turnOnAllLED()));

        // CommandScheduler.getInstance().schedule(ledCommand.ignoringDisable(true));

        // ledManager.turnOffAllLED();
    }

    private void configureBindings() {

    }
    
    
    public Command getAutonomousCommand()
    {
        return chooser.get();
    }


}
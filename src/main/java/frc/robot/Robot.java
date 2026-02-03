// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Shooter.ShotCalculator;
import frc.utils.ModeFileHandling;
import frc.utils.SwitchedToPitModeException;
import io.github.captainsoccer.basicmotor.motorManager.MotorManager;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;


public class Robot extends LoggedRobot
{
    private Command autonomousCommand;
    



    public Robot()
    {
//        Logger.recordMetadata("ProjectName", "*GENERIC_ROBOT_PROJECT*"); // Set a metadata value

        if (isReal()) {
            Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
            Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        } else {
            Logger.addDataReceiver(new NT4Publisher());
        }

        
        Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.

        RobotContainer.getInstance();

        
        CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());
    }


    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        MotorManager.getInstance().periodic(); // must run AFTER CommandScheduler

    }
    
    
    @Override
    public void disabledInit() {}
    
    
    @Override
    public void disabledPeriodic() {

        //Check if should switch to pit mode
        if(ModeFileHandling.isCompMode() && ModeFileHandling.shouldSwitchToPitMode()){
            ModeFileHandling.switchToPitMode();
            throw new SwitchedToPitModeException("Switched to pit mode");
        }
    }
    
    
    @Override
    public void disabledExit() {}
    
    
    @Override
    public void autonomousInit()
    {
        autonomousCommand = RobotContainer.getInstance().getAutonomousCommand();
        
        if (autonomousCommand != null)
        {
            CommandScheduler.getInstance().schedule(autonomousCommand);
        }
    }
    
    
    @Override
    public void autonomousPeriodic() {}


    @Override
    public void autonomousExit() {}
    
    
    @Override
    public void teleopInit()
    {
        if (autonomousCommand != null)
        {

            autonomousCommand.cancel();
            Constants.HubTiming.setStartingTeam(DriverStation.getGameSpecificMessage(), DriverStation.getAlliance().get());

        }

    }
    
    
    @Override
    public void teleopPeriodic() {
        ShotCalculator.getInstance().clearShootingParameters();
    }
    
    
    @Override
    public void teleopExit() {}
    
    
    @Override
    public void testInit()
    {
        CommandScheduler.getInstance().cancelAll();
    }
    
    
    @Override
    public void testPeriodic() {}
    
    
    @Override
    public void testExit() {}




}

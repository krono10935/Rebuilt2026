// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.revrobotics.util.StatusLogger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.HubTiming;
import frc.robot.leds.LED;
import frc.robot.leds.LEDConstants;
import frc.robot.leds.PatternFactory;
import frc.robot.subsystems.Shooter.ShotCalculator;
import frc.utils.Elastic;
import frc.utils.ModeFileHandling;
import frc.utils.SwitchedToPitModeException;
import frc.utils.VirtualSubSystem;
import io.github.captainsoccer.basicmotor.motorManager.MotorManager;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;


public class Robot extends LoggedRobot
{
    private Command autonomousCommand;


    public Robot()
    {
        initializeLogging(ModeFileHandling.isCompMode());

        new Trigger(()-> DriverStation.isDSAttached()).onTrue(new InstantCommand(() -> Elastic.selectTab("Autonomous")));

        RobotContainer.getInstance();
    }

    private void initializeLogging(boolean isOnField){
        SignalLogger.enableAutoLogging(false);
        SignalLogger.stop();

        StatusLogger.disableAutoLogging();
        StatusLogger.stop();

        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        Logger.recordMetadata(
            "GitDirty",
            switch (BuildConstants.DIRTY) {
            case 0 -> "All changes committed";
            case 1 -> "Uncommitted changes";
            default -> "Unknown";
            });

         // TODO comment out before comp
         if (isReal()) {
             Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
             Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
         } else {
             Logger.addDataReceiver(new NT4Publisher());
         }

        //TODO comment in before comp
          // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

        Logger.start();
    }


    @Override
    public void robotPeriodic() {
        VirtualSubSystem.virtualperiodic();
        CommandScheduler.getInstance().run();
        MotorManager.getInstance().periodic(); // must run AFTER CommandScheduler
        ShotCalculator.getInstance().clearShootingParameters();

        // if (HubTiming.getAutoIsActiveDetection() == null && !DriverStation.getGameSpecificMessage().isEmpty() && DriverStation.getAlliance().isPresent()){
        //     HubTiming.setStartingTeam(DriverStation.getGameSpecificMessage(), DriverStation.getAlliance().get());
        // }
    }
    
    
    @Override
    public void disabledInit() {}
    
    private DriverStation.Alliance lastAlliance = DriverStation.Alliance.Red;
    @Override
    public void disabledPeriodic() {
        //Check if should switch to pit mode
        if(ModeFileHandling.isCompMode() && ModeFileHandling.shouldSwitchToPitMode()){
            ModeFileHandling.switchToPitMode();
            throw new SwitchedToPitModeException("Switched to pit mode");
        }

        var alliance = DriverStation.getAlliance();

        if(alliance.isPresent() && alliance.get() != lastAlliance){
            lastAlliance = alliance.get();

            LED.getInstance().setPattern(LEDConstants.Segments.ALL, PatternFactory.defaultPattern(lastAlliance));
        }
    }
    
    
    @Override
    public void disabledExit() {
        RobotContainer.getInstance().drivetrain.setBrakeMode(true);
    }
    
    
    @Override
    public void autonomousInit()
    {
        autonomousCommand = RobotContainer.getInstance().getAutonomousCommand();

        if(DriverStation.getAlliance().isPresent()){
            LED.getInstance().setPattern(LEDConstants.Segments.ALL,
                    PatternFactory.defaultPattern(DriverStation.getAlliance().get()));
        }
        
        if (autonomousCommand != null)
        {
            CommandScheduler.getInstance().schedule(autonomousCommand);
            
        }
    }
    
    
    @Override
    public void autonomousPeriodic() {}


    @Override
    public void autonomousExit() {
        HubTiming.setStartingTeam("R", DriverStation.getAlliance().get());
    }
    
    
    @Override
    public void teleopInit()
    {
        if (autonomousCommand != null)
        {

            autonomousCommand.cancel();
        }

        RobotContainer.getInstance().drivetrain.reset(RobotContainer.getInstance().drivetrain.getEstimatedPosition());

    }
    
    
    @Override
    public void teleopPeriodic() {}
    
    
    @Override
    public void teleopExit() {
        RobotContainer.getInstance().drivetrain.setBrakeMode(false);
        LED.getInstance().turnOffLed();
        LED.getInstance().setPattern(LEDConstants.Segments.ALL, LEDPattern.rainbow(255, 255), 3);
    }
    
    
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

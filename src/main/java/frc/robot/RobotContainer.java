// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.FieldConstants.TowerSide;
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
import frc.robot.subsystems.Shooter.ShotCalculator;
import frc.robot.subsystems.Shooter.IO.ShootRealConstants;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.Vision.ObjectDetection.ObjectDetection;
import frc.robot.subsystems.Vision.VisionConstants.CamerasConstants;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.drivetrain.PPController;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.conduit.ConduitApi;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;


public class RobotContainer
{   
    private static RobotContainer instance;

    // public final LedManager ledManager;

    public final Vision vision;

    public final Shooter shooter;

    public final Intake intake;

    public final ObjectDetection objectDetector;

    // public final Climb climb;

    private final CommandXboxController xboxController;

    public final Drivetrain drivetrain;

     private final LoggedDashboardChooser<Command> autoChooser;

    private final LoggedDashboardChooser<FieldConstants.TowerSide> climbChooser; 

    private final LoggedNetworkNumber shooterSpeedMPS;

    private final LoggedNetworkNumber hoodAngle;


    public static RobotContainer getInstance(){
        if (instance == null){
            instance = new RobotContainer();
        }
        return instance;
    }

    private RobotContainer()
    {

        drivetrain = new Drivetrain(ConduitApi.getInstance()::getPDPVoltage, Constants.CHASSIS_TYPE.constants);

        shooter = new Shooter();


        intake = new Intake();

        // climb = new Climb();

        xboxController = new CommandXboxController(0);

        vision = new Vision(drivetrain::addVisionMeasurement, drivetrain::getEstimatedPosition);
        vision.setCamAsPriority(CamerasConstants.SHOOTER_CAMERA);

        objectDetector = ObjectDetection.getInstance();

        autoChooser = new LoggedDashboardChooser<>("Auto", AutoBuilder.buildAutoChooser());

        autoChooser.onChange(this::displayChosenAuto);


        climbChooser = new LoggedDashboardChooser<>("Climb side");
        climbChooser.addOption("Left",TowerSide.left);
        climbChooser.addOption("Right", TowerSide.right);

        shooterSpeedMPS = new LoggedNetworkNumber("shooterSpeedMPS", 10);

        SmartDashboard.putNumber("shooterSpeedMPS", 10);

        hoodAngle = new LoggedNetworkNumber("hoodAngle", 1);

        SmartDashboard.putNumber("hoodAngle", 1);

        // ledManager = new LedManager();
        // configureBindings();
        configureTestBindings();
    }

    private void configureTestBindings(){
        // xboxController.a().onTrue(new SpinUp(shooter, drivetrain).alongWith(new OpenCommand(intake))
        //     .andThen(new ShootCommand(shooter, drivetrain, intake, vision)))
        // .onFalse(new InstantCommand(() -> {
        //     shooter.stopFlyWheel();
        //     shooter.setHoodAngle(Rotation2d.kZero);
        //     shooter.toggleKicker(false);
        //     shooter.getIndexer().turnOff();
        // }, shooter, drivetrain, shooter.getIndexer()).alongWith(new CloseCommand(intake)));

        // drivetrain.setDefaultCommand(new DriveCommand(drivetrain, xboxController));

        xboxController.b().onTrue(new RunCommand(() -> ShotCalculator.getInstance()
            .getParameters(drivetrain.getEstimatedPosition(), drivetrain.getChassisSpeeds())));


        xboxController.a().onTrue(new InstantCommand(() ->  {
            shooter.spinUp(SmartDashboard.getNumber("shooterSpeedMPS", 10));
            shooter.setHoodAngle(Rotation2d.fromDegrees(SmartDashboard.getNumber("hoodAngle", 1)));
        }, shooter)
            .andThen(new WaitUntilCommand(() -> shooter.isHoodAtSetpoint() && shooter.isShooterAtGoal()), 
            shooter.getIndexer().turnOnIndexerCommand(),
            new InstantCommand(() -> shooter.toggleKicker(true),shooter),
            new RunCommand(() -> shooter.keepVelocity(shooterSpeedMPS.getAsDouble()), shooter)));
        
        xboxController.a().onFalse(new InstantCommand(() -> {
            shooter.stopFlyWheel();
            shooter.setHoodAngle(Rotation2d.fromDegrees(ShootRealConstants.getHoodMotorConfig().constraintsConfig.minValue));
            shooter.toggleKicker(false);
            shooter.getIndexer().turnOff();
        }, shooter));

        // xboxController.a().whileTrue(new IntakeCommand(intake));

    }

    private void configurePitBindings() {
        xboxController.a().onTrue(new SpinUp(shooter, drivetrain)
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
            // climb.getCurrentCommand().cancel();
            // CommandScheduler.getInstance().schedule(climb.idle());
        }));

        xboxController.y().debounce(0.3).whileTrue(new OpenCommand(intake));

        xboxController.y().onFalse(new CloseCommand(intake));

        xboxController.x().onTrue(new IntakeCommand(intake).onlyIf(intake::isOpen));

        // xboxController.povDown().onTrue(climb.closeCommand());

        // xboxController.povUp().onTrue(climb.openCommand());

        xboxController.povLeft().whileTrue(shooter.idle());

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

        // xboxController.povUp().whileTrue(Sequences.autoClimb(intake, drivetrain, climb, climbChooser::get, shooter,vision));

        // xboxController.b().whileTrue(climb.closeCommand());

        xboxController.x().onTrue(
            Sequences.delivery(drivetrain, shooter, xboxController,intake));

        xboxController.a().onTrue(drivetrain.resetGyro());
    }
    public Command getAutonomousCommand()
    {
        return autoChooser.get();
    }

    private void displayChosenAuto(Command command){
        if(RobotState.isEnabled()){
            drivetrain.clearFiledPath();
            return;
        }

        List<PathPlannerPath> auto;
        try{
            auto = PathPlannerAuto.getPathGroupFromAutoFile(command.getName());
        }
        catch(IOException | ParseException e){
            Logger.recordOutput("autoDisplay", e.getMessage());
            drivetrain.clearFiledPath();
            return;
        }

        ArrayList<Pose2d> poses = new ArrayList<>();
        for(PathPlannerPath path : auto){
            poses.addAll(path.getPathPoses());
        }
        
        drivetrain.addPathToField(poses);
    }

    public void registerNamedCommand(DriveAndHomeCommand driveAndHomeCommand){

        Command aimRobot = new StartEndCommand(() -> {
            PPController.setThetaOverride(driveAndHomeCommand::calculateThetaPID);
        }, PPController::clearThetaOverride);

        Command aimRobotStationary = new RunCommand(
                () -> drivetrain.drive(new ChassisSpeeds(
                        0, 0, driveAndHomeCommand.calculateThetaPID())), drivetrain);


        NamedCommands.registerCommand("shootAndAimMoving",
                ShootCommand.shootCommandFactory(shooter, drivetrain, xboxController,intake, vision).beforeStarting(new SpinUp(shooter, drivetrain))
                        .alongWith(aimRobot));

        NamedCommands.registerCommand("shootAndAimStationary",
                ShootCommand.shootCommandFactory(shooter, drivetrain, xboxController,intake, vision).beforeStarting(new SpinUp(shooter, drivetrain))
                        .alongWith(aimRobotStationary));

        NamedCommands.registerCommand("spinUp", new SpinUp(shooter, drivetrain));

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
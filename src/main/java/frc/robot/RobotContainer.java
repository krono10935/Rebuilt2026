// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.IntakeCommands.*;
import frc.robot.subsystems.drivetrain.configsStructure.ChassisConstants;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.conduit.ConduitApi;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.DriveToPoseConstants;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.FieldConstants.Hub;
import frc.robot.FieldConstants.TowerSide;
import frc.robot.commands.Drivetrain.DriveAndHomeCommand;
import frc.robot.commands.Drivetrain.DriveCommand;
import frc.robot.commands.Drivetrain.DriveRobotRelative;
import frc.robot.commands.Drivetrain.SwerveSysID;
import frc.robot.commands.Shooter.ShootCommand;
import frc.robot.commands.Shooter.SpinUp;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Shooter.ShotCalculator;
import frc.robot.subsystems.Shooter.IO.ShootRealConstants;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.Vision.VisionConstants.CamerasConstants;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.Vision.ObjectDetection.ObjectDetection;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.PPController;
import frc.robot.subsystems.intake.Intake;


public class RobotContainer
{
    private static RobotContainer instance;

    // public final LedManager ledManager;

    public final Vision vision;

    public final Shooter shooter;

    public final Intake intake;

    // public final Climb climb;

    private final CommandXboxController driverController;

    private final GenericHID operatorController;

    public final Drivetrain drivetrain;

    private final LoggedDashboardChooser<Command> autoChooser;

    private final LoggedDashboardChooser<FieldConstants.TowerSide> climbChooser;


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

        driverController = new CommandXboxController(0);

        operatorController = new GenericHID(1);

        vision = new Vision(drivetrain::addVisionMeasurement, drivetrain::getEstimatedPosition);
        vision.setCamAsPriority(CamerasConstants.SHOOTER_CAMERA);

        autoChooser = registerNamedCommand(new DriveAndHomeCommand(drivetrain, driverController));

        climbChooser = new LoggedDashboardChooser<>("Climb side");
        climbChooser.addDefaultOption("Left",TowerSide.left);
        climbChooser.addOption("Right", TowerSide.right);


        ObjectDetection.getInstance();

        DriveToPoseConstants.MAX_LINEAR_SPEED = 4.5;
        DriveToPoseConstants.POSE_TOLERANCE = 0.01;


        // ledManager = new LedManager();
        // configureBindings();
        test();
    }

    private void test(){
        driverController.a().toggleOnTrue(
            ShootCommand.shootCommandFactory(shooter, drivetrain, driverController, intake, vision));
        driverController.b().onTrue( Sequences.intakeOpenStart(intake));
        driverController.x().onTrue(Sequences.stopIntakeAndClose(intake));
        drivetrain.setDefaultCommand(new DriveCommand(drivetrain,driverController));

        LoggedNetworkNumber posPercent = new LoggedNetworkNumber("IntakePercent", -0.05);

        driverController.y().onTrue(IntakeFactory.resetIntake(intake));

        driverController.rightBumper().onTrue(drivetrain.resetGyro());



    }



    private void configurePitBindings() {

        // Disable all subsystems commands
        driverController.b().onTrue(new InstantCommand(() -> {
            drivetrain.getCurrentCommand().cancel();
            CommandScheduler.getInstance().schedule(drivetrain.idle());
            shooter.getCurrentCommand().cancel();
            CommandScheduler.getInstance().schedule(shooter.idle());
            intake.getCurrentCommand().cancel();
            CommandScheduler.getInstance().schedule(intake.idle());
            // climb.getCurrentCommand().cancel();
            // CommandScheduler.getInstance().schedule(climb.idle());
        }));

        driverController.y().debounce(0.3).whileTrue(new OpenCommand(intake));

        driverController.y().onFalse(new CloseCommand(intake));

        driverController.x().whileTrue(new IntakeCommand(intake).onlyIf(intake::isOpen));

        // xboxController.povDown().onTrue(climb.closeCommand());

        // xboxController.povUp().onTrue(climb.openCommand());

        driverController.rightBumper().onTrue(drivetrain.resetGyro());
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(new DriveCommand(drivetrain, driverController));

        BooleanSupplier isHubActive = () -> {
            double time = DriverStation.getMatchTime();

            return Constants.HubTiming.isActive(time) ||
                    Constants.HubTiming.isActive(time - Constants.HUB_ACTIVITY_DEABAND_BEFORE_ACTIVE) ||
                    Constants.HubTiming.isActive(time + Constants.HUB_ACTIVITY_DEABAND_AFTER_ACTIVE);
        };
        new Trigger(isHubActive).
                and(() -> FieldConstants.isInAllianceZone(drivetrain.getEstimatedPosition()))
                .whileTrue(ShootCommand.shootCommandFactory(shooter ,drivetrain ,driverController, intake, vision));

        driverController.y().toggleOnTrue(Sequences.intakeOpenStart(intake).alongWith(new DriveRobotRelative(drivetrain, driverController)));

        // xboxController.povUp().whileTrue(Sequences.autoClimb(intake, drivetrain, climb, climbChooser::get, shooter,vision));

        // xboxController.b().whileTrue(climb.closeCommand()); 

        driverController.x().onTrue(
                Sequences.delivery(drivetrain, shooter, driverController,intake));

        driverController.a().onTrue(drivetrain.resetGyro());

        Trigger povUp = new Trigger(() -> 
            Math.abs(
                operatorController.getPOV() - ShooterConstants.ANGLE_UP.getDegrees()
            ) < ShooterConstants.POV_TOLERANCE.getDegrees());

        Trigger povRight = new Trigger(() -> 
            Math.abs(
                operatorController.getPOV() - ShooterConstants.ANGLE_RIGHT.getDegrees()
            ) < ShooterConstants.POV_TOLERANCE.getDegrees());

        Trigger povDown = new Trigger(() -> 
            Math.abs(
                operatorController.getPOV() - ShooterConstants.ANGLE_DOWN.getDegrees()
            ) < ShooterConstants.POV_TOLERANCE.getDegrees());

        Trigger povLeft = new Trigger(() -> 
            Math.abs(
                operatorController.getPOV() - ShooterConstants.ANGLE_LEFT.getDegrees()
            ) < ShooterConstants.POV_TOLERANCE.getDegrees());

        povUp.onTrue(new InstantCommand(() -> 
            ShootCommand.AddToHoodOffset(
                ShooterConstants.HOOD_ANGLE_OFFSET_PER_CLICK
                )
            )
        );

        povDown.onTrue(new InstantCommand(() -> 
            ShootCommand.AddToHoodOffset(
                ShooterConstants.HOOD_ANGLE_OFFSET_PER_CLICK.unaryMinus()
                )
            )
        );

        povRight.onTrue(new InstantCommand(() -> 
            ShootCommand.AddToFlywheelOffset(
                ShooterConstants.SHOOT_SPEED_MPS_OFFSET_PER_CLICK
                )
            )
        );

        povLeft.onTrue(new InstantCommand(() -> 
            ShootCommand.AddToFlywheelOffset(
                -ShooterConstants.SHOOT_SPEED_MPS_OFFSET_PER_CLICK
                )
            )
        );

        Trigger button1 = new Trigger(() -> operatorController.getRawButton(1));

        button1.onTrue(new InstantCommand(() -> ShootCommand.setOverrideObjectDetection(true)))
        .onFalse(new InstantCommand(() -> ShootCommand.setOverrideObjectDetection(false)));

        
        // Trigger closeEnoughToSpinUp = new Trigger(() -> drivetrain.getEstimatedPosition().)
    }
    public Command getAutonomousCommand()
    {
        return autoChooser.get().beforeStarting(IntakeFactory.resetIntake(intake));
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
            path = ChassisConstants.shouldFlipPath() ? path : path.flipPath();
            poses.addAll(path.getPathPoses());
        }

        drivetrain.addPathToField(poses);
    }

    public LoggedDashboardChooser<Command> registerNamedCommand(DriveAndHomeCommand driveAndHomeCommand){

        Command aimRobot = new StartEndCommand(() -> {
            PPController.setThetaOverride(driveAndHomeCommand::calculateThetaPID);
        }, PPController::clearThetaOverride);

        Command aimRobotStationary = new RunCommand(
                () -> drivetrain.drive(new ChassisSpeeds(
                        0, 0, driveAndHomeCommand.calculateThetaPID())), drivetrain);


        NamedCommands.registerCommand("shootAndAimMoving",
                new ShootCommand(shooter, drivetrain, vision).beforeStarting(new SpinUp(shooter, drivetrain))
                        .alongWith(aimRobot));

        NamedCommands.registerCommand("shootAndAimStationary",
                new ShootCommand(shooter, drivetrain, vision).beforeStarting(new SpinUp(shooter, drivetrain))
                        .alongWith(aimRobotStationary));

        NamedCommands.registerCommand("spinUp", new SpinUp(shooter, drivetrain));

        NamedCommands.registerCommand("waitUntilNoBalls", new WaitUntilCommand(() ->
                !ObjectDetection.getInstance().hasBalls()).andThen(new WaitCommand(0.3))
                .andThen(Commands.print("no balls")));

         NamedCommands.registerCommand("openIntake",
                 new SequentialCommandGroup(Sequences.intakeOpenStart(intake)));
         NamedCommands.registerCommand("closeIntake",
                 new SequentialCommandGroup(Sequences.stopIntakeAndClose(intake)));

        // NamedCommands.registerCommand("openClimb", climb.openCommand());
        // NamedCommands.registerCommand("closeClimb", climb.closeCommand());

        LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto", AutoBuilder.buildAutoChooser());
        autoChooser.onChange(this::displayChosenAuto);
        return autoChooser;
    }
}
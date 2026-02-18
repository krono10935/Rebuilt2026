// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.FieldConstants.TowerSide;
import frc.robot.commands.DriveAndHomeCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.DriveRobotRelative;
import frc.robot.commands.IntakeCommands.CloseCommand;
import frc.robot.commands.IntakeCommands.IntakeCommand;
import frc.robot.commands.IntakeCommands.OpenCommand;
import frc.robot.commands.Shooter.ShootCommand;
import frc.robot.commands.Shooter.SpinUp;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Shooter.ShooterSysID;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.climb.Climb;
import frc.robot.leds.LedManager;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.drivetrain.PPController;
import org.littletonrobotics.conduit.ConduitApi;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class RobotContainer
{   
    private static RobotContainer instance;

    // public final LedManager ledManager;

    // public final Vision vision;

    public final Shooter shooter;

    // public final Intake intake;

    // public final Climb climb;

    private final CommandXboxController xboxController;

    // public final Drivetrain drivetrain;

    // private final LoggedDashboardChooser<Command> autoChooser;

    // private final LoggedDashboardChooser<FieldConstants.TowerSide> climbChooser; 

    private final LoggedNetworkNumber shooterSpeedMPS;

    // private final LoggedNetworkNumber hoodAngleDegrees;

    // private final Indexer indexer;

    private final ShooterSysID sysID;


    public static RobotContainer getInstance(){
        if (instance == null){
            instance = new RobotContainer();
        }
        return instance;
    }

    private RobotContainer()
    {

        shooter = new Shooter();

        sysID = new ShooterSysID(shooter);
        // indexer = new Indexer();

        // intake = new Intake();

        // climb = new Climb();

        xboxController = new CommandXboxController(0);

        // drivetrain = new Drivetrain(ConduitApi.getInstance()::getPDPVoltage, Constants.CHASSIS_TYPE.constants);

        // vision = new Vision(drivetrain::addVisionMeasurement, drivetrain::getEstimatedPosition);

        // autoChooser = new LoggedDashboardChooser<>("Auto", AutoBuilder.buildAutoChooser());

        // climbChooser = new LoggedDashboardChooser<>("Climb side");
        // climbChooser.addOption("Left",TowerSide.left);
        // climbChooser.addOption("Right", TowerSide.right);

        shooterSpeedMPS = new LoggedNetworkNumber("shooterSpeedMPS", 0.2);

        // hoodAngleDegrees = new LoggedNetworkNumber("hoodAngleDegrees", 0);

        // ledManager = new LedManager();
        // configureBindings();
        configureTestBindings();
    }

    private void configureTestBindings(){
        // xboxController.a().onTrue(new InstantCommand(() -> {
        //         shooter.spinUp(shooterSpeedMPS.get());
        //         shooter.setHoodAngle(Rotation2d.fromDegrees(hoodAngleDegrees.get()));
        //         shooter.toggleKicker(true);
        //         shooter.getIndexer().turnOn();
        //     }, shooter ,shooter.getIndexer())
        // .withDeadline(new WaitUntilCommand(
        //     () -> shooter.isHoodAtSetpoint() 
        //     && shooter.isShooterAtGoal()))
        // .andThen(new RunCommand(() -> shooter.keepVelocity(shooterSpeedMPS.get()),shooter)));

        // xboxController.b().onTrue(new InstantCommand(() -> {
        //     shooter.stopFlyWheel();
        //     shooter.setHoodAngle(Rotation2d.kZero);
        //     shooter.toggleKicker(false);
        //     shooter.getIndexer().turnOff();
        // }, shooter, shooter.getIndexer()));

        // xboxController.x().onTrue(new InstantCommand(() -> shooter.spinUp(shooterSpeedMPS.get())));
        // xboxController.x().onFalse(new InstantCommand(() -> shooter.stopFlyWheel()));

        // xboxController.y().onTrue(new InstantCommand(() -> shooter.dutyCycle(shooterSpeedMPS.get())));
        // xboxController.y().onFalse(new InstantCommand(() -> shooter.stopFlyWheel()));

        // xboxController.b().onTrue(new InstantCommand(() -> shooter.toggleKicker(true)));
        // xboxController.b().onFalse(new InstantCommand(() -> shooter.toggleKicker(false)));

        // xboxController.a().onTrue(shooter.getIndexer().turnOnIndexerCommand());
        // xboxController.a().onFalse(shooter.getIndexer().turnOffIndexerCommand());

        xboxController.a().onTrue(sysID.sysIdDynamicFlywheel(Direction.kForward));
        xboxController.b().onTrue(sysID.sysIdDynamicFlywheel(Direction.kReverse));
        xboxController.y().onTrue(sysID.sysIdQuasistaticFlywheel(Direction.kForward));
        xboxController.x().onTrue(sysID.sysIdQuasistaticFlywheel(Direction.kReverse));



        // xboxController.povLeft().onTrue(new InstantCommand(() -> 
        //     intake.setPosition(IntakeConstants.OPEN_POSITION), intake));
        
        // xboxController.povRight().onTrue(new IntakeCommand(intake));

        // xboxController.povLeft().onFalse(new InstantCommand(() -> intake.setPosition(IntakeConstants.CLOSE_POSITION), intake));

        // xboxController.povRight().onFalse(new InstantCommand(() ->
        //     intake.stopIntakeMotor()));

        // xboxController.povCenter().onTrue(climb.openCommand());
        // xboxController.povCenter().onFalse(climb.closeCommand());



    }

    private void configurePitBindings() {
        // xboxController.a().onTrue(new SpinUp(shooter)
        // .until(shooter::isShooterAtGoal)
        // .andThen(new RunCommand(() -> shooter.keepVelocity(ShooterConstants.SHOOTING_SPEED), shooter)));

        // Disable all subsystems commands
        // xboxController.b().onTrue(new InstantCommand(() -> {
        //     drivetrain.getCurrentCommand().cancel();
        //     CommandScheduler.getInstance().schedule(drivetrain.idle());
        //     shooter.getCurrentCommand().cancel();
        //     CommandScheduler.getInstance().schedule(shooter.idle());
        //     intake.getCurrentCommand().cancel();
        //     CommandScheduler.getInstance().schedule(intake.idle());
        //     climb.getCurrentCommand().cancel();
        //     CommandScheduler.getInstance().schedule(climb.idle());
        // }));

        // xboxController.y().debounce(0.3).whileTrue(new OpenCommand(intake));

        // xboxController.y().onFalse(new CloseCommand(intake));

        // xboxController.x().onTrue(new IntakeCommand(intake).onlyIf(intake::isOpen));

        // xboxController.povDown().onTrue(climb.closeCommand());

        // xboxController.povUp().onTrue(climb.openCommand());

        // xboxController.povLeft().whileTrue(shooter.idle());

        // xboxController.povCenter().onTrue(drivetrain.resetGyro());
    }

    private void configureBindings() {
        // drivetrain.setDefaultCommand(new DriveCommand(drivetrain, xboxController));

        // BooleanSupplier isHubActive = () -> {
        //     double time = DriverStation.getMatchTime();

        //     return Constants.HubTiming.isActive(time) ||
        //             Constants.HubTiming.isActive(time - Constants.HUB_ACTIVITY_DEABAND_BEFORE_ACTIVE) ||
        //             Constants.HubTiming.isActive(time + Constants.HUB_ACTIVITY_DEABAND_AFTER_ACTIVE);
        // };
    //    new Trigger(isHubActive).
    //            and(() -> FieldConstants.isInAllianceZone(drivetrain.getEstimatedPosition()))
    //     .whileTrue(ShootCommand.shootCommandFactory(shooter,drivetrain,xboxController,intake, vision));

        // xboxController.y().toggleOnTrue(Sequences.intakeOpenStart(intake).alongWith(new DriveRobotRelative(drivetrain, xboxController)));

        // xboxController.povUp().whileTrue(Sequences.autoClimb(intake, drivetrain, climb, climbChooser::get, shooter,vision));

        // xboxController.b().whileTrue(climb.closeCommand());

        // xboxController.x().onTrue(
        //     Sequences.delivery(drivetrain, shooter, xboxController,intake));

        // xboxController.a().onTrue(drivetrain.resetGyro());
    }
    public Command getAutonomousCommand()
    {
        return null;//autoChooser.get();
    }

    public void registerNamedCommand(DriveAndHomeCommand driveAndHomeCommand){

        // Command aimRobot = new StartEndCommand(() -> {
        //     PPController.setThetaOverride(driveAndHomeCommand::calculateThetaPID);
        // }, PPController::clearThetaOverride);

        // Command aimRobotStationary = new RunCommand(
        //         () -> drivetrain.drive(new ChassisSpeeds(
        //                 0, 0, driveAndHomeCommand.calculateThetaPID())), drivetrain);


    //     NamedCommands.registerCommand("shootAndAimMoving",
    //             ShootCommand.shootCommandFactory(shooter, drivetrain, xboxController,intake, vision).beforeStarting(new SpinUp(shooter))
    //                     .alongWith(aimRobot));

    //     NamedCommands.registerCommand("shootAndAimStationary",
    //             ShootCommand.shootCommandFactory(shooter, drivetrain, xboxController,intake, vision).beforeStarting(new SpinUp(shooter))
    //                     .alongWith(aimRobotStationary));

    //     NamedCommands.registerCommand("spinUp", new SpinUp(shooter));

    //     NamedCommands.registerCommand("waitUntilNoBalls", new WaitUntilCommand(() ->
    //             !new Intake().hasBalls()));

    //     NamedCommands.registerCommand("openIntake",
    //             new SequentialCommandGroup(Sequences.intakeOpenStart(intake)));
    //     NamedCommands.registerCommand("closeIntake",
    //             new SequentialCommandGroup(Sequences.stopIntakeAndClose(intake)));

    //     NamedCommands.registerCommand("openClimb", new Climb().openCommand());
    //     NamedCommands.registerCommand("closeClimb", new Climb().closeCommand());
    }


}
// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

import org.json.simple.parser.ParseException;
import org.littletonrobotics.conduit.ConduitApi;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.FieldConstants.TowerSide;
import frc.robot.commands.DriveAndHomeCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.DriveRobotRelative;
import frc.robot.commands.SwerveSysID;
import frc.robot.commands.IntakeCommands.CloseCommand;
import frc.robot.commands.IntakeCommands.IntakeCommand;
import frc.robot.commands.IntakeCommands.OpenCommand;
import frc.robot.commands.IntakeCommands.ShakeItOffCommand;
import frc.robot.commands.Shooter.ShootCommand;
import frc.robot.commands.Shooter.SpinUp;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.IO.ShootRealConstants;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.Vision.VisionConstants.CamerasConstants;
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

        registerNamedCommand(new DriveAndHomeCommand(drivetrain, xboxController));

        autoChooser = new LoggedDashboardChooser<>("Auto", AutoBuilder.buildAutoChooser());

        autoChooser.onChange(this::displayChosenAuto);


        climbChooser = new LoggedDashboardChooser<>("Climb side");
        climbChooser.addOption("Left",TowerSide.left);
        climbChooser.addOption("Right", TowerSide.right);

        shooterSpeedMPS = new LoggedNetworkNumber("shooterSpeedMPS", 10);

        SmartDashboard.putNumber("shooterSpeedMPS", 10);

        hoodAngle = new LoggedNetworkNumber("hoodAngle", 1);

        SmartDashboard.putNumber("hoodAngle", 1);

        ObjectDetection.getInstance();

        // ledManager = new LedManager();
        // configureBindings();
        testShooter();
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

        // xboxController.b().onTrue(new RunCommand(() -> ShotCalculator.getInstance()
        //     .getParameters(drivetrain.getEstimatedPosition(), drivetrain.getChassisSpeeds())));


        xboxController.a().onTrue(new InstantCommand(() ->  {
            shooter.spinUp(SmartDashboard.getNumber("shooterSpeedMPS", 0));
            shooter.setHoodAngle(Rotation2d.fromDegrees(SmartDashboard.getNumber("hoodAngle", 30)));
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
;
        // xboxController.rightBumper().toggleOnTrue(new DriveCommand(drivetrain,xboxController));
        xboxController.leftBumper().onTrue(drivetrain.resetGyro());
        xboxController.b().toggleOnTrue(new DriveAndHomeCommand(drivetrain, xboxController));
        drivetrain.setDefaultCommand(new DriveCommand(drivetrain, xboxController));

        // xboxController.a().whileTrue(new IntakeCommand(intake));

        // xboxController.a().onTrue(shooter.getIndexer().turnOnIndexerCommand().alongWith(new ShakeItOffCommand(intake)));
        // xboxController.b().onTrue(shooter.getIndexer().turnOffIndexerCommand().alongWith(new CloseCommand(intake)));

    }

    private void testShooter(){

        LoggedNetworkNumber speed = new LoggedNetworkNumber("Shake/rollerSpeed", 0.3);
        // xboxController.a().whileTrue((new SpinUp(shooter, drivetrain)
        //     .andThen(new InstantCommand(() ->
        //     shooter.setHoodAngle(Rotation2d.fromDegrees(SmartDashboard.getNumber("hoodAngle", 1))), shooter)))
            
        //     .andThen(shooter.getIndexer().turnOnIndexerCommand(), new InstantCommand(() -> {
        //     shooter.toggleKicker(true);
        //     intake.setIntakeMotorVelocity(speed.get());
        // })
        // , new RunCommand(() -> shooter.keepVelocity(SmartDashboard.getNumber("shooterSpeedMPS", 0)), shooter).alongWith(new ShakeItOffCommand(intake))));

        // xboxController.a().onFalse(new InstantCommand(() -> {
        //     shooter.stopFlyWheel();
        //     shooter.getIndexer().turnOff();
        //     shooter.setHoodAngle(Rotation2d.fromDegrees(1));
        //     intake.stopIntakeMotor();
        // }, shooter, shooter.getIndexer()));

        xboxController.y().onTrue(new OpenCommand(intake));
        xboxController.b().toggleOnTrue(new DriveCommand(drivetrain, xboxController));
        xboxController.x().onTrue(new CloseCommand(intake));
        xboxController.leftBumper().onTrue(new InstantCommand(intake::resetEncoder));

        xboxController.a().toggleOnTrue((
                (new ShootCommand(shooter, drivetrain, intake, vision).alongWith(new InstantCommand(() -> intake.setPercent(speed.getAsDouble())))
            .   alongWith(new ShakeItOffCommand(intake))).
            beforeStarting(new SpinUp(shooter, drivetrain)).alongWith(new DriveAndHomeCommand(drivetrain, xboxController)))
        );

        xboxController.rightBumper().onTrue(new InstantCommand(drivetrain::resetGyro));
        // xboxController.a().toggleOnTrue(new DriveAndHomeCommand(drivetrain, xboxController));
        // drivetrain.setDefaultCommand(new DriveCommand(drivetrain, xboxController));
        // xboxController.b().toggleOnTrue(ShootCommand.shootCommandFactory(shooter, drivetrain, xboxController, intake, vision));
        // xboxController.y().whileTrue(new IntakeCommand(intake));
        // xboxController.leftBumper().onTrue(drivetrain.resetGyro());

        // xboxController.rightBumper().toggleOnTrue(new DriveRobotRelative(drivetrain, xboxController));



    }

    private void configurePitBindings() {
        // xboxController.a().onTrue(new SpinUp(shooter, drivetrain)
        // .until(shooter::isShooterAtGoal)
        // .andThen(Kee, shooter)));

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

    private void configureBindingsSysid(){
        SwerveSysID sysID = new SwerveSysID(drivetrain, xboxController);
        // xboxController.a().whileTrue(sysID.sysIdDynamicDrive(Direction.kForward));
        // xboxController.b().whileTrue(sysID.sysIdDynamicDrive(Direction.kReverse));
        // xboxController.y().whileTrue(sysID.sysIdQuasistaticDrive(Direction.kForward));
        // xboxController.x().whileTrue(sysID.sysIdQuasistaticDrive(Direction.kReverse));
        // Rotation2d[] arr = {Rotation2d.kZero, Rotation2d.kZero, Rotation2d.kZero, Rotation2d.kZero};
        //xboxController.rightBumper().onTrue(new InstantCommand(() -> drivetrain.setDriveVoltageAndSteerAngle(0, arr)));
        xboxController.a().whileTrue(sysID.sysIdDynamicSpin(Direction.kForward));
        xboxController.b().whileTrue(sysID.sysIdDynamicSpin(Direction.kReverse));
        xboxController.x().whileTrue(sysID.sysIdQuasistaticSpin(Direction.kForward));
        xboxController.y().whileTrue(sysID.sysIdQuasistaticSpin(Direction.kReverse));
        xboxController.leftBumper().onTrue(drivetrain.resetGyro());
        drivetrain.setDefaultCommand(new DriveCommand(drivetrain, xboxController));
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

    public Command registerNamedCommand(DriveAndHomeCommand driveAndHomeCommand){

        Command aimRobot = new StartEndCommand(() -> {
            PPController.setThetaOverride(driveAndHomeCommand::calculateThetaPID);
        }, PPController::clearThetaOverride);

        Command aimRobotStationary = new RunCommand(
                () -> drivetrain.drive(new ChassisSpeeds(
                        0, 0, driveAndHomeCommand.calculateThetaPID())), drivetrain);


        NamedCommands.registerCommand("shootAndAimMoving",
                new ShootCommand(shooter, drivetrain,intake, vision).beforeStarting(new SpinUp(shooter, drivetrain))
                        .alongWith(aimRobot));

        NamedCommands.registerCommand("shootAndAimStationary",
                new ShootCommand(shooter, drivetrain,intake, vision).beforeStarting(new SpinUp(shooter, drivetrain))
                        .alongWith(aimRobotStationary));

        NamedCommands.registerCommand("spinUp", new SpinUp(shooter, drivetrain));

        NamedCommands.registerCommand("waitUntilNoBalls", new WaitUntilCommand(() ->
                !ObjectDetection.getInstance().hasBalls()).andThen(Commands.print("no balls")));

        // NamedCommands.registerCommand("openIntake",
        //         new SequentialCommandGroup(Sequences.intakeOpenStart(intake)));
        // NamedCommands.registerCommand("closeIntake",
        //         new SequentialCommandGroup(Sequences.stopIntakeAndClose(intake)));

        // NamedCommands.registerCommand("openClimb", climb.openCommand());
        // NamedCommands.registerCommand("closeClimb", climb.closeCommand());
        return autoChooser.get();
    }

    private void testIntake(){
        xboxController.a().onTrue(new InstantCommand(() -> intake.setPosition(0.3)));
        xboxController.b().onTrue(new InstantCommand(() -> intake.setPosition(0)));
        xboxController.x().onTrue(new InstantCommand(intake::resetEncoder).ignoringDisable(true));
    }


}
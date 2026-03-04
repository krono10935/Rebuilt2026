// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.*;
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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.FieldConstants.TowerSide;
import frc.robot.commands.Drivetrain.DriveAndHomeCommand;
import frc.robot.commands.Drivetrain.DriveCommand;
import frc.robot.commands.Drivetrain.DriveRobotRelative;
import frc.robot.commands.Drivetrain.SwerveSysID;
import frc.robot.commands.IntakeCommands.CloseCommand;
import frc.robot.commands.IntakeCommands.IntakeCommand;
import frc.robot.commands.IntakeCommands.OpenCommand;
import frc.robot.commands.IntakeCommands.ShakeItOffCommand;
import frc.robot.commands.Shooter.ShootCommand;
import frc.robot.commands.Shooter.SpinUp;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterConstants;
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

    private final CommandXboxController driverController;

    private final GenericHID operatorController;

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

        driverController = new CommandXboxController(0);

        operatorController = new GenericHID(1);

        vision = new Vision(drivetrain::addVisionMeasurement, drivetrain::getEstimatedPosition);
        vision.setCamAsPriority(CamerasConstants.SHOOTER_CAMERA);

        autoChooser = registerNamedCommand(new DriveAndHomeCommand(drivetrain, driverController));

        climbChooser = new LoggedDashboardChooser<>("Climb side");
        climbChooser.addOption("Left",TowerSide.left);
        climbChooser.addOption("Right", TowerSide.right);

        shooterSpeedMPS = new LoggedNetworkNumber("shooterSpeedMPS", 10);

        SmartDashboard.putNumber("shooterSpeedMPS", 10);

        hoodAngle = new LoggedNetworkNumber("hoodAngle", 1);

        SmartDashboard.putNumber("hoodAngle", 1);

        ObjectDetection.getInstance();

        DriveToPoseConstants.MAX_LINEAR_SPEED = 4.5;
        DriveToPoseConstants.POSE_TOLERANCE = 0.01;
        SmartDashboard.putData(DriveToPoseConstants.ANGULAR_PID_GAINS);
        SmartDashboard.putData(DriveToPoseConstants.LINEAR_PID_GAINS);



    

        // ledManager = new LedManager();
        // configureBindings();
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


        driverController.a().onTrue(new InstantCommand(() ->  {
            shooter.spinUp(SmartDashboard.getNumber("shooterSpeedMPS", 0));
            shooter.setHoodAngle(Rotation2d.fromDegrees(SmartDashboard.getNumber("hoodAngle", 30)));
        }, shooter)
                .andThen(new WaitUntilCommand(() -> shooter.isHoodAtSetpoint() && shooter.isShooterAtGoal()),
                        shooter.getIndexer().turnOnIndexerCommand(),
                        new InstantCommand(() -> shooter.toggleKicker(true),shooter),
                        new RunCommand(() -> shooter.keepVelocity(shooterSpeedMPS.getAsDouble()), shooter)));

        driverController.a().onFalse(new InstantCommand(() -> {
            shooter.stopFlyWheel();
            shooter.setHoodAngle(Rotation2d.fromDegrees(ShootRealConstants.getHoodMotorConfig().constraintsConfig.minValue));
            shooter.toggleKicker(false);
            shooter.getIndexer().turnOff();
        }, shooter));
        ;
        // xboxController.rightBumper().toggleOnTrue(new DriveCommand(drivetrain,xboxController));
        driverController.leftBumper().onTrue(drivetrain.resetGyro());
        driverController.b().toggleOnTrue(new DriveAndHomeCommand(drivetrain, driverController));
        drivetrain.setDefaultCommand(new DriveCommand(drivetrain, driverController));

        // xboxController.a().whileTrue(new IntakeCommand(intake));

        // xboxController.a().onTrue(shooter.getIndexer().turnOnIndexerCommand().alongWith(new ShakeItOffCommand(intake)));
        // xboxController.b().onTrue(shooter.getIndexer().turnOffIndexerCommand().alongWith(new CloseCommand(intake)));

    }

    private void testIntake(){
        driverController.a().onTrue(Sequences.intakeOpenStart(intake));
        driverController.b().onTrue(Sequences.stopIntakeAndClose(intake));
        drivetrain.setDefaultCommand(new DriveCommand(drivetrain, driverController));
        driverController.y().onTrue(drivetrain.resetGyro());
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

        driverController.y().onTrue(new OpenCommand(intake));
        driverController.b().toggleOnTrue(new DriveCommand(drivetrain, driverController));
        driverController.x().onTrue(new CloseCommand(intake));
        driverController.leftBumper().onTrue(new InstantCommand(intake::resetEncoder));
        drivetrain.setDefaultCommand(new DriveCommand(drivetrain, driverController));
        driverController.a().toggleOnTrue((
                (new ShootCommand(shooter, drivetrain, intake, vision).alongWith(new InstantCommand(() -> intake.setPercent(speed.getAsDouble())))
                        .   alongWith(new ShakeItOffCommand(intake))).
                        beforeStarting(new SpinUp(shooter, drivetrain)).alongWith(new DriveAndHomeCommand(drivetrain, driverController)))
        );

        driverController.rightBumper().onTrue(new InstantCommand(drivetrain::resetGyro));
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

        driverController.x().onTrue(new IntakeCommand(intake).onlyIf(intake::isOpen));

        // xboxController.povDown().onTrue(climb.closeCommand());

        // xboxController.povUp().onTrue(climb.openCommand());
        driverController.povLeft().whileTrue(shooter.idle());

        driverController.povCenter().onTrue(drivetrain.resetGyro());
    }

    private void configureBindingsSysid(){
        SwerveSysID sysID = new SwerveSysID(drivetrain, driverController);
        // xboxController.a().whileTrue(sysID.sysIdDynamicDrive(Direction.kForward));
        // xboxController.b().whileTrue(sysID.sysIdDynamicDrive(Direction.kReverse));
        // xboxController.y().whileTrue(sysID.sysIdQuasistaticDrive(Direction.kForward));
        // xboxController.x().whileTrue(sysID.sysIdQuasistaticDrive(Direction.kReverse));
        // Rotation2d[] arr = {Rotation2d.kZero, Rotation2d.kZero, Rotation2d.kZero, Rotation2d.kZero};
        //xboxController.rightBumper().onTrue(new InstantCommand(() -> drivetrain.setDriveVoltageAndSteerAngle(0, arr)));
        driverController.a().whileTrue(sysID.sysIdDynamicSpin(Direction.kForward));
        driverController.b().whileTrue(sysID.sysIdDynamicSpin(Direction.kReverse));
        driverController.x().whileTrue(sysID.sysIdQuasistaticSpin(Direction.kForward));
        driverController.y().whileTrue(sysID.sysIdQuasistaticSpin(Direction.kReverse));
        driverController.leftBumper().onTrue(drivetrain.resetGyro());
        drivetrain.setDefaultCommand(new DriveCommand(drivetrain, driverController));
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

        // xboxController.b().whileTrue(climb.closeCommand()); // TODO fix to actually do climb.

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
                ShooterConstants.HOOD_ANGLE_OFFSET_PER_CLICK, false
                )
            )
        );

        povDown.onTrue(new InstantCommand(() -> 
            ShootCommand.AddToHoodOffset(
                ShooterConstants.HOOD_ANGLE_OFFSET_PER_CLICK, true
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
                new ShootCommand(shooter, drivetrain,intake, vision).beforeStarting(new SpinUp(shooter, drivetrain))
                        .alongWith(aimRobot));

        NamedCommands.registerCommand("shootAndAimStationary",
                new ShootCommand(shooter, drivetrain,intake, vision).beforeStarting(new SpinUp(shooter, drivetrain))
                        .alongWith(aimRobotStationary));

        NamedCommands.registerCommand("spinUp", new SpinUp(shooter, drivetrain));

        NamedCommands.registerCommand("waitUntilNoBalls", new WaitUntilCommand(() ->
                !ObjectDetection.getInstance().hasBalls()).andThen(new WaitCommand(0.3))
                .andThen(Commands.print("no balls")));

        // NamedCommands.registerCommand("openIntake",
        //         new SequentialCommandGroup(Sequences.intakeOpenStart(intake)));
        // NamedCommands.registerCommand("closeIntake",
        //         new SequentialCommandGroup(Sequences.stopIntakeAndClose(intake)));

        // NamedCommands.registerCommand("openClimb", climb.openCommand());
        // NamedCommands.registerCommand("closeClimb", climb.closeCommand());

        LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto", AutoBuilder.buildAutoChooser());
        autoChooser.onChange(this::displayChosenAuto);
        return autoChooser;
    }
}
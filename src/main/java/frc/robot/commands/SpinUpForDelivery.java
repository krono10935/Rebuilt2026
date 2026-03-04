// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.utils.AllianceFlipUtil;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SpinUpForDelivery extends Command {
  private final Drivetrain drivetrain;
  private final Shooter shooter;
  private final double maxShootingSpeedMPS;

  private final double MIN_DELIVERY_DISTANCE = 2; //m

  /** Creates a new SpinUpForDelivery. */
  public SpinUpForDelivery(Drivetrain drivetrain, Shooter shooter, double MaxSpinUpSpeedMPS){
                           // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.shooter = shooter;
    this.maxShootingSpeedMPS = MaxSpinUpSpeedMPS;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.spinUp(scale()* maxShootingSpeedMPS);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double distance = Math.abs(drivetrain.getEstimatedPosition().getX() - AllianceFlipUtil.apply(FieldConstants.Hub.farLeftCorner).getX());
    return MIN_DELIVERY_DISTANCE < distance && shooter.isShooterAtGoal();
  }

  /**
   *
   * @return scalar to scale the spinUp speed based on distance from hub
   */
  private double scale(){
    double distance = Math.abs(drivetrain.getEstimatedPosition().getX() - AllianceFlipUtil.apply(FieldConstants.Hub.farLeftCorner).getX());
    //make sure min is not spinning up at all when too far
    return Math.max((MIN_DELIVERY_DISTANCE - distance) , 0) / MIN_DELIVERY_DISTANCE;
  }
}

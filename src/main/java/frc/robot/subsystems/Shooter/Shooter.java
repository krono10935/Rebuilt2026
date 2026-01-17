// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private ShooterIO io;

  public Shooter() {
    io = new ShooterIOReal();

  }

  public void shoot(){
    io.shoot(ShooterConstants.SHOOTING_SPEED);
  }

  public void stop(){
    io.stop();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

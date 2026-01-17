// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private ShooterIO io;

  public Shooter() {
    io = RobotBase.isReal() ? new ShooterIOReal() : new ShooterIOSim();
  }

  public void shoot(int pidSlot){
    io.shoot(ShooterConstants.SHOOTING_SPEED, pidSlot);
  }

  public void stop(){
    io.stop();
  }

  @Override
  public void periodic() {
    io.update();
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Shooter.ShooterIO.ShooterIOReplay;
import frc.robot.subsystems.Shooter.IO.ShooterIOReal;
import frc.robot.subsystems.Shooter.IO.ShooterIOSim;

public class Shooter extends SubsystemBase {

    private final ShooterIO io;

    private final Indexer indexer;

    private final ShooterInputsAutoLogged inputs;


    private boolean isKeepingVelocity;

    /**
     * Create a shooter IO based on the robot's state (sim, dev, comp)
     */
    public Shooter() {

        indexer = new Indexer();

        switch (Constants.currentMode) {
            case REAL -> io = new ShooterIOReal();

            case SIM -> io = new ShooterIOSim();
        
            default -> io = new ShooterIOReplay();
        }

        inputs = new ShooterInputsAutoLogged();
    }


    @Override
    public void periodic() {

        io.update(inputs);

        Logger.processInputs(getName(), inputs);

        Logger.recordOutput("Shooter/current command", getCurrentCommand() == null ? "None" : getCurrentCommand().getName());
        Logger.recordOutput("Shooter/is hood at setpoint", isHoodAtSetpoint());
        Logger.recordOutput("Shooter/is shooter at setpoint", isShooterAtGoal());
    }

    /**
     * Log for sysid
     */
    public void logSysID() {
        io.logSysID();
    }

    /**
     *
     * @param speedMPS speed to spinUp to
     */
    public void spinUp(double speedMPS) {
        io.spinUp(speedMPS);
        isKeepingVelocity = false;
    }

    public void keepVelocity(double speedMPS) {
        io.keepVelocity(speedMPS);
        isKeepingVelocity = true;
    }

    /**
     * Stop the flywheel
     */
    public void stopFlyWheel() {
        io.stopFlyWheel();
        isKeepingVelocity = false;
    }

    /**
     *
     * @return whether the shooter is at setpoint
     */
    public boolean isShooterAtGoal() {
        return inputs.isFlywheelAtGoal;
    }

    /**
     *
     * @param voltage apply this voltage to the flywheel motor(s)
     */
    public void setVoltage(double voltage) {
        io.setFlyWheelVoltage(voltage);
    }

    /**
     *
     * @param isActive toggle on or off the kicker
     */
    public void toggleKicker(boolean isActive) {
        io.toggleKicker(isActive);
    }

    /**
     * @return whether or not the kicker is active
     */
    public boolean isKickerActive() {
        return !inputs.isKickerStuck;
    }

    /**
     *
     * @return whether the hood is at setpoint
     */
    public boolean isHoodAtSetpoint() {
        return inputs.isHoodAtSetpoint;
    }

    /**
     *
     * @param angle angle to set the hood to
     */
    public void setHoodAngle(Rotation2d angle) {
        io.setHoodAngle(angle);
    }

    /**
     * @return whether the shooter is ready to shoot
     */
    public boolean readyToShoot() {
        return isHoodAtSetpoint() && isShooterAtGoal();
    }

    /**
     * @return the indexer (the subsystem is coupled with shooter)
     */
    public Indexer getIndexer() {
        return indexer;
    }

    /**
     * @param dutyCycle apply this duty cycle to the flywheel
     */
    public void dutyCycle(double dutyCycle) {
        io.setFlyWheelVoltage(dutyCycle * 12);
    }

    /**
     * @return whether the shooter is keeping velocity
     */
    public boolean isKeepingVelocity() {
        return isKeepingVelocity;
    }

    /**
     * @return A command to reset the shooter and indexer
     */
    public Command resetShooterCommand() {
        return new InstantCommand(() -> {
            stopFlyWheel();
            setHoodAngle(Rotation2d.fromDegrees(1));
            toggleKicker(false);
            getIndexer().turnOff();
        }, this, getIndexer());
    }

    public double getShooterVelocity(){
        return inputs.shooterSpeedMPS;
    }

}

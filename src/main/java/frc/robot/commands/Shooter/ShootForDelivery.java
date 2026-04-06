package frc.robot.commands.Shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.Shooter;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ShootForDelivery extends Command {

    private final Shooter shooter;

    private LoggedNetworkNumber flyWheelSpeed = new LoggedNetworkNumber("Delivery/FlyWheelSpeed", 15);

    private LoggedNetworkNumber hoodAngle = new LoggedNetworkNumber("Delivery/HoodAngle", 28);


    public ShootForDelivery(Shooter shooter){
        this.shooter = shooter;

        addRequirements(shooter,shooter.getIndexer());
    }

    @Override
    public void initialize(){
        shooter.setHoodAngle(Rotation2d.fromDegrees(hoodAngle.get()));
        shooter.spinUp(flyWheelSpeed.get());
        shooter.getIndexer().spinForward();
        shooter.toggleKicker(true);
    }

    @Override
    public void end(boolean interrupted){
        shooter.stopFlyWheel();
        shooter.toggleKicker(false);
        shooter.getIndexer().turnOff();
    }
}

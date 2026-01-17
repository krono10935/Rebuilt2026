package frc.robot.subsystems.Shooter;

public interface ShooterIO {
    void shoot(double speed, int pidSlot);
    void stop();
    void update();
}

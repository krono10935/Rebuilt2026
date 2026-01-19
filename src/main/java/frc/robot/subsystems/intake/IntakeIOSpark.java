package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.DigitalInput;
import io.github.captainsoccer.basicmotor.BasicMotor;
import io.github.captainsoccer.basicmotor.rev.BasicSparkMAX;

public class IntakeIOSpark implements IntakeIO {
    private final BasicMotor motor;
    private final DigitalInput beamBreak;

    public IntakeIOSpark() {
        motor = new BasicSparkMAX(IntakeConstants.intakeMotorConfig);
        beamBreak = new DigitalInput(IntakeConstants.BEAM_BREAK_CHANNEL);
    }

    @Override
    public boolean getBeamBrake() {
        return beamBreak.get();
    }

    @Override
    public void stopMotor() {
        motor.stop();
    }

    @Override
    public double getMotorPower() {
        return IntakeConstants.MOTOR_POWER_PRECENT;
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        inputs.temp = motor.getSensorData().temperature();
    }

    

}

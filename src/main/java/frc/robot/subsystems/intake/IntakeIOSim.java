package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.DigitalInput;
import io.github.captainsoccer.basicmotor.BasicMotor;
import io.github.captainsoccer.basicmotor.sim.motor.BasicMotorSim;

public class IntakeIOSim implements IntakeIO {
 private final BasicMotor intakeMotor;
 private final BasicMotor openCloseMotor;
    private final DigitalInput beamBreak;

    public IntakeIOSim() {
        intakeMotor = new BasicMotorSim(IntakeConstants.intakeMotorConfig);
        openCloseMotor = new BasicMotorSim(IntakeConstants.openCloseMotorConfig);
        beamBreak = new DigitalInput(IntakeConstants.BEAM_BREAK_CHANNEL);
    }

    @Override
    public boolean getBeamBrake() {
        return beamBreak.get();
    }

    @Override
    public void stopMotor() {
        intakeMotor.stop();
    }

    @Override
    public double getMotorPower() {
        return IntakeConstants.MOTOR_POWER_PRECENT;
    }

    @Override
    public double getPos(){
        return openCloseMotor.getPosition();
    }

    @Override
    public void setPos(double pos){
        openCloseMotor.resetEncoder(pos);
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        inputs.temp = intakeMotor.getSensorData().temperature();
    }

}

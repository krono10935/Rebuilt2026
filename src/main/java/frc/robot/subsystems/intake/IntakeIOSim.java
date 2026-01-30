package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotState;
import io.github.captainsoccer.basicmotor.BasicMotor;
import io.github.captainsoccer.basicmotor.controllers.Controller.ControlMode;
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
    public void setPercentOutput(double percentOutput){
        intakeMotor.setPercentOutput(percentOutput);

    }


    @Override
    public Rotation2d getPos(){
        return Rotation2d.fromRotations(openCloseMotor.getPosition());
    }

    @Override
    public void setActivationMotorPos(Rotation2d pos){
        openCloseMotor.setControl(pos.getRotations(), ControlMode.POSITION);
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        inputs.Angle = Rotation2d.fromRotations(openCloseMotor.getPosition());
        inputs.power = IntakeConstants.INTAKE_KT 
            * intakeMotor.getSensorData().currentOutput()
            * Units.rotationsPerMinuteToRadiansPerSecond(intakeMotor.getVelocity());
        inputs.velocity = intakeMotor.getVelocity(); 
    }

}

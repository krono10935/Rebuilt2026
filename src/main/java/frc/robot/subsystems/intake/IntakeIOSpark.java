package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import io.github.captainsoccer.basicmotor.BasicMotor;
import io.github.captainsoccer.basicmotor.controllers.Controller.ControlMode;
import io.github.captainsoccer.basicmotor.rev.BasicSparkMAX;

public class IntakeIOSpark implements IntakeIO {
    private final BasicMotor intakeMotor;
    private final BasicMotor positionMotor;
    private final DigitalInput beamBreak;
    private final DigitalInput limitSwitch;

    public IntakeIOSpark() {

        intakeMotor = new BasicSparkMAX(IntakeConstants.intakeMotorConfig);

        positionMotor = new BasicSparkMAX(IntakeConstants.positionMotorConfig);

        beamBreak = new DigitalInput(IntakeConstants.BEAM_BREAK_CHANNEL);

        limitSwitch = new DigitalInput(IntakeConstants.LIMIT_SWITCH_CHANNEL);

    }


    @Override
    public boolean getBeamBrake() {
        return beamBreak.get();
    }

    @Override
    public boolean getLimitSwitch(){
        return limitSwitch.get();
    }

    @Override
    public void stopMotor() {
        intakeMotor.stop();
    }

    @Override
    public void setVelocityOutput(Rotation2d pos){
        intakeMotor.setControl(pos.getRotations(), ControlMode.VELOCITY);

    }

    @Override
    public double getPos(){
        return positionMotor.getPosition();
    }

    @Override
    public void setPositionMotor(double pos){
        positionMotor.setControl(pos, ControlMode.POSITION);
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        inputs.Angle = Rotation2d.fromRotations(positionMotor.getPosition());
        inputs.power = IntakeConstants.INTAKE_KT 
            * intakeMotor.getSensorData().currentOutput()
            * Units.rotationsPerMinuteToRadiansPerSecond(intakeMotor.getVelocity());
        inputs.velocity = intakeMotor.getVelocity(); 
    }


    @Override
    public boolean intakeAtSetPoint() {
       return intakeMotor.atSetpoint();
    }


    @Override
    public boolean positionAtSetPoint() {
        return positionMotor.atSetpoint();
    }

    

}

package frc.robot.subsystems.climb;

import frc.robot.subsystems.climb.ClimbConstants.ClimbState;
import io.github.captainsoccer.basicmotor.controllers.Controller.ControlMode;
import io.github.captainsoccer.basicmotor.sim.motor.BasicMotorSim;

public class ClimbIOSim implements ClimbIO{
    
    private final BasicMotorSim motor;
    
    private ClimbState state;

    public ClimbIOSim(){
        motor = new BasicMotorSim(ClimbConstants.getClimbConfig());
        state = ClimbState.CLOSED;
    }

    @Override
    public ClimbState getClimbState() {
        return state;
    }

    @Override
    public void close() {
        motor.setControl(ClimbConstants.CLOSED_ANGLE.getRotations(), ControlMode.POSITION);
        state = ClimbState.CLOSING;
    }

    @Override
    public void open() {
        motor.setControl(ClimbConstants.OPENED_ANGLE.getRotations(), ControlMode.POSITION);
        state = ClimbState.OPENING;
    }

    @Override
    public boolean isAtSetPoint() {
        return motor.atSetpoint();
    }

    @Override
    public void update(ClimbInputs inputs) {

        switch(state){
            case CLOSING:
                if(isAtSetPoint()) state = ClimbState.CLOSED;
            case OPENING:
                if(isAtSetPoint()) state = ClimbState.OPEN;
            default:
        }

        inputs.state = this.state;
    }

}

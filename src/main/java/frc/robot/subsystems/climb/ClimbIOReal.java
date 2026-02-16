package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.climb.ClimbConstants.ClimbState;
import io.github.captainsoccer.basicmotor.BasicMotor;
import io.github.captainsoccer.basicmotor.controllers.Controller.ControlMode;
import io.github.captainsoccer.basicmotor.rev.BasicSparkMAX;

public class ClimbIOReal implements ClimbIO{
    
    private final BasicMotor motor;
    
    private ClimbState state;

    public ClimbIOReal(){
        motor = new BasicSparkMAX(ClimbConstants.getClimbConfig());

        SmartDashboard.putData(motor.getController());

        state = ClimbState.CLOSED;
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
    public void stop(){
        motor.stop();
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

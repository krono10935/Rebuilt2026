package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

import java.util.function.DoubleSupplier;

public class publishTimeToNextShift extends Command {

    private final DoubleSupplier timeSupplier;

    public publishTimeToNextShift(DoubleSupplier timeSupplier){
        this.timeSupplier = timeSupplier;
    }

    @Override
    public void execute(){
        if(Constants.HubTiming.isActive(timeSupplier.getAsDouble())){
            SmartDashboard.putNumber("Time to next shift", -1);
        }
        else{
            SmartDashboard.putNumber("Time to next shift", Constants.HubTiming.timeRemainingToShift(timeSupplier.getAsDouble()));
        }
    }


}

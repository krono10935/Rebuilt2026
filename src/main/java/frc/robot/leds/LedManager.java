package frc.robot.leds;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;


/**
 * Manages the robot LED state by publishing it to NetworkTables.
 * <p>
 */
public class LedManager extends SubsystemBase {
    /**
     * A list of all the strips connected to the LED controller
     */
    private final LEDStrip strip;

    /**
     * The status of the rsl light, used to sync the leds with the rsl.
     */
    private final NetworkTableEntry rslStatus;

    private final PowerDistribution pdh;

    /**
     * Creates a new LED manager
     */
    public LedManager() {
        rslStatus = NetworkTableInstance.getDefault().getTable("Led").getEntry("RslStatus");

        strip = new LEDStrip(18);

        pdh = new PowerDistribution();
        turnOnAllLED();
    }

    /**
     * publishes the colors to networkTable
     * including the location of the leds to set, the pattern chosen, the colors to use for the pattern
     * and updates the entry to say that a new command for the leds has been chosen.
     * @param state the LED state to activate for the robot
     */
    public void setColors(LedState state){
        strip.addPattern(state);
    }

    public void turnOffAllLED(){
        pdh.setSwitchableChannel(false);
    }

    public void turnOnAllLED(){
        pdh.setSwitchableChannel(true);
    }

    @Override
    public void periodic(){
        if(Robot.isReal()) rslStatus.setBoolean(RobotController.getRSLState());
        else Logger.runEveryN(25, () -> rslStatus.setBoolean(!rslStatus.getBoolean(false)));
        
        strip.publish();
    }

}

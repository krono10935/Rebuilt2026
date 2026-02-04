package frc.robot.leds;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;

import java.util.ArrayList;

public class LEDStrip {
    /**
     * The publisher for the LED states
     */
    private final StructArrayPublisher<LedState> publisher;

    /**
     * The array holding the next states of the LED
     */
    private final ArrayList<LedState> list = new ArrayList<>();

    /**
     * The ID if this LED strip
     */
    public final int id;

    /**
     * Creates an LED Strip with the ID
     * @param id The ID of the strip (The GPIO port on the pi)
     */
    public LEDStrip(int id){
        var nt = NetworkTableInstance.getDefault();

        this.id = id;

        publisher = nt.getTable("Led").getStructArrayTopic("Strip" + id, LedState.struct).publish();
    }

    /**
     * Adds the pattern to the strip
     * @param state the pattern to add
     */
    public void addPattern(LedState state){
        list.add(state);
    }

    /**
     * Publishes the pattern list to the network tables and clears the current list
     */
    public void publish(){
        if(list.isEmpty()) return;

        publisher.set(list.toArray(new LedState[0]));

        list.clear();
    }
}

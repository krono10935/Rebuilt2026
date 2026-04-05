

package frc.robot.leds;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.AddressableLED.ColorOrder;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.HashMap;

public class LED extends SubsystemBase {
    private final AddressableLED led;
    private final AddressableLEDBuffer buffer;

    private final HashMap<LEDConstants.Segments, LedSegment> segments = new HashMap<>();

    private static LED instance;

    public static LED getInstance() {
        if (instance == null) {
            instance = new LED();
        }

        return instance;
    }

    /**
     * Creates a new LED.
     */
    private LED() {
        led = new AddressableLED(LEDConstants.LED_PORT);
        led.setLength(LEDConstants.LED_COUNT_TOTAL);
        led.setColorOrder(ColorOrder.kBGR);

        buffer = new AddressableLEDBuffer(LEDConstants.LED_COUNT_TOTAL);

        for(var segment : LEDConstants.Segments.values()){
            var view = buffer.createView(segment.start, segment.end);
            segments.put(segment, new LedSegment(view));
        }

        setPattern(LEDConstants.Segments.ALL, PatternFactory.defaultPattern(DriverStation.Alliance.Blue));

        led.setData(buffer);
        led.start();
    }

    @Override
    public void periodic() {
        // for(var segment : segments.values()){
        //     segment.apply();
        // }
        LEDPattern.solid(Color.kGreen).applyTo(buffer);

        led.setData(buffer);
    }

    public void setPattern(LEDConstants.Segments segment, LEDPattern pattern, double timeout){
        segments.get(segment).setPattern(pattern, timeout);
    }

    public void setPattern(LEDConstants.Segments segment, LEDPattern pattern){
        setPattern(segment, pattern, 0);
    }
}
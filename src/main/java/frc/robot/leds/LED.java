

package frc.robot.leds;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.AddressableLED.ColorOrder;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.HashMap;
import java.util.Optional;

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
        led.setColorOrder(ColorOrder.kRGB);
        setPattern(LEDConstants.Segments.ALL,LEDPattern.kOff);
        buffer = new AddressableLEDBuffer(LEDConstants.LED_COUNT_TOTAL);

        for(var segment : LEDConstants.Segments.values()){
            var view = buffer.createView(segment.start, segment.end);
            segments.put(segment, new LedSegment(view));
        }

        setPattern(LEDConstants.Segments.ALL, PatternFactory.defaultPattern(DriverStation.getAlliance()));
//      TEST PATTERN FOR CONSTANTS
//        setPattern(LEDConstants.Segments.RIO, PatternFactory.solid(Color.kRed, Units.Percent.of(100)));
//        setPattern(LEDConstants.Segments.PDH_RIGHT,PatternFactory.solid(Color.kBlue, Units.Percent.of(100)));
//        setPattern(LEDConstants.Segments.PDH_LEFT,PatternFactory.solid(Color.kRed, Units.Percent.of(100)));
//        setPattern(LEDConstants.Segments.INDEXER, PatternFactory.solid(Color.kBlue, Units.Percent.of(100)));
//        setPattern(LEDConstants.Segments.INTAKE, PatternFactory.solid(Color.kRed, Units.Percent.of(100)));

        setPattern(LEDConstants.Segments.INTAKE, PatternFactory.shooterSpunUpIndicator(false));

        led.setData(buffer);
        led.start();
    }

    @Override
    public void periodic() {
        for(var segment : segments.values()){
             segment.apply();
        }

        led.setData(buffer);
    }

    public void setPattern(LEDConstants.Segments segment, LEDPattern pattern, double timeout){
        segments.get(segment).setPattern(pattern, timeout);
    }

    public void setPattern(LEDConstants.Segments segment, LEDPattern pattern){
        setPattern(segment, pattern, 0);
    }
}
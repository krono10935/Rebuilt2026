package frc.robot.leds;

import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;

public class LedSegment {
    private final AddressableLEDBufferView view;

    private LEDPattern pattern = LEDPattern.kOff;
    private double expiryTime = 0;

    public LedSegment(AddressableLEDBufferView view){
        this.view = view;
    }

    public void apply(){
        if(expiryTime != 0 && expiryTime < Timer.getTimestamp()){
            pattern = LEDPattern.kOff;
        }
        pattern.applyTo(view);
    }

    public void setPattern(LEDPattern pattern, double timeout){
        this.pattern = pattern;
        if(timeout != 0)
            expiryTime = Timer.getTimestamp() + timeout;
        else
            expiryTime = 0;
    }

    public void setPattern(LEDPattern pattern){
        setPattern(pattern, 0);
    }

    public void clearPattern(){
        setPattern(LEDPattern.kOff);
    }
}

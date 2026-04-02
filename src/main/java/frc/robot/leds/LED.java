

package frc.robot.leds;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import java.util.Map;
import java.util.function.DoubleSupplier;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.leds.LEDConstants.AllainceColor;

public class LED extends SubsystemBase {
    AddressableLED led;
    AddressableLEDBuffer buffer;
    StripControl controlType = StripControl.ALL;

    AddressableLEDBufferView bufferFrontHalf;
    AddressableLEDBufferView bufferBackHalf;

    LEDPattern defaultPattern;
    LEDPattern pattern;

    LEDPattern closeFeederPattern;
    LEDPattern awayFeederPattern;

    boolean ledOn = true;

    public enum StripControl{
        ALL,
        HALF
    }

    /** Creates a new LED. */
    public LED() {
        led = new AddressableLED(LEDConstants.LED_PORT);
        led.setLength(LEDConstants.LED_COUNT_TOTAL);

        buffer = new AddressableLEDBuffer(LEDConstants.LED_COUNT_TOTAL);

        bufferFrontHalf = buffer.createView(0, LEDConstants.LED_COUNT_TOTAL / 2 - 1);
        bufferBackHalf = buffer.createView(LEDConstants.LED_COUNT_TOTAL / 2, LEDConstants.LED_COUNT_TOTAL - 1);

        pattern = LEDPattern.solid(Color.kDarkRed);
        pattern.applyTo(buffer);

        led.setData(buffer);
        led.start();
    }

    public void setLEDState(boolean isOn){
        ledOn = isOn;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        switch (controlType){
            case ALL:
                pattern.applyTo(buffer);
                break;
            case HALF:
                pattern.applyTo(bufferFrontHalf);
                pattern.reversed().applyTo(bufferBackHalf);
                break;
        }

        if(!ledOn) LEDPattern.kOff.applyTo(buffer);

        led.setData(buffer);
    }

    public void setStripControl(StripControl control){
        controlType = control;
    }

    public InstantCommand setStripControlCommand(StripControl control){
        return new InstantCommand(() -> setStripControl(control));
    }


    public void setDefaultPattern(boolean isRedAlliance){

        AllainceColor allainceColor = isRedAlliance ? AllainceColor.RED : AllainceColor.BLUE;

        Color color = allainceColor.MOVING_COLOR;
        Color backgroundColor = allainceColor.BACKGORUND_COLOR;

        Dimensionless colorBrightness = allainceColor.COLOR_BRIGHTNESS;
        Dimensionless backgorundBrihtness = allainceColor.BACKGROUND_BRIGHTNESS;

        LEDPattern pattern1 = LEDPattern.steps(Map.of(0, color, 0.05, Color.kBlack))
                .scrollAtRelativeSpeed(Percent.of(LEDConstants.DEFAULT_SCROLL_SPEED).per(Second))
                .atBrightness(colorBrightness);

        LEDPattern whitePattern = LEDPattern.solid(backgroundColor).atBrightness(backgorundBrihtness);

        LEDPattern gradiant = pattern1.overlayOn(pattern1.reversed());

        defaultPattern = whitePattern;
    }

    public void putDefaultPattern(){
        pattern = defaultPattern;
    }

    public InstantCommand putDefaultPatternCommand(){
        InstantCommand command = new InstantCommand(this::putDefaultPattern);

        command.addRequirements(this);

        return command;
    }

    public void setSolidColour(Color colour){
        pattern = LEDPattern.solid(colour);
    }

    public InstantCommand SetSolidColourCommand(Color colour){
        InstantCommand command = new InstantCommand(() -> setSolidColour(colour));

        command.addRequirements(this);

        return command;
    }

    public void Blink(double seconds){
        pattern = pattern.blink(Time.ofBaseUnits(seconds, Second));
    }

    public void Blink(double onSeconds, double offSeconds){
        pattern = pattern.blink(Time.ofBaseUnits(onSeconds, Second), Time.ofBaseUnits(offSeconds, Second));
    }

    public InstantCommand BlinkCommand(double seconds){
        InstantCommand command = new InstantCommand(() -> Blink(seconds));

        command.addRequirements(this);

        return command;
    }

    public void blinkWithRSL(Color color){
        pattern = LEDPattern.solid(color).synchronizedBlink(RobotController::getRSLState);
    }

    public Command blinkWithRSLCommand(Color color){
        InstantCommand command = new InstantCommand(() -> blinkWithRSL(color));

        command.addRequirements(this);

        return command;
    }

    public InstantCommand BlinkCommand(double onSeconds, double offSeconds){
        return new InstantCommand(() -> Blink(onSeconds, offSeconds));
    }

    public void setGradient(Color... colours){
        pattern = LEDPattern.gradient(GradientType.kDiscontinuous, colours);
    }

    public void setMovingGradient(int speedPercent, Color... colours){
        pattern = LEDPattern.gradient(GradientType.kContinuous, colours).scrollAtRelativeSpeed(Percent.per(Second).of(speedPercent));
    }

    public void setRainbow(){
        pattern = LEDPattern.rainbow(255, 255);
    }

    public void setMovingRainbow(int speedPercent){
        pattern = LEDPattern.rainbow(256, 256).scrollAtRelativeSpeed(Percent.per(Second).of(speedPercent));
    }

    public void setProgressLayer(DoubleSupplier percent){
        pattern = pattern.mask(LEDPattern.progressMaskLayer(percent));
    }

    public InstantCommand setProgressLayerCOmmand(DoubleSupplier percent){
        return new InstantCommand(() -> setProgressLayer(percent));
    }
}
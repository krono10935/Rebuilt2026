package frc.robot.leds;

/**
 * Possible patterns for the leds, how they look visually is determined on the server side
 */
public enum LedPattern {
    RAINBOW,
    SOLID,
    BLINK,
    RSL_BLINK,
    SKEBOB,
    CAPTAIN_USA,
    BRWON,
    BLUE_PULSE;

    @Override
    public String toString(){
        return this.name().toLowerCase();
    }
}

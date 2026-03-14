package frc.utils.controllers;

public enum ControllerMultiplierType {
    LINEAR(1),
    SQRT(0.5),
    FORUTH_ROOT(0.25);

    public final double exponent;

    ControllerMultiplierType(double exponent){
        this.exponent = exponent;
    }
}

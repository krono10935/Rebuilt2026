package frc.utils.controllers;

import edu.wpi.first.wpilibj.GenericHID;

public class ExponentialHID extends GenericHID {

    public ExponentialHID(int port) {
        super(port);

    }

    @Override
    public double getRawAxis(int axis) {
        return Math.sqrt(super.getRawAxis(axis));
    }
}

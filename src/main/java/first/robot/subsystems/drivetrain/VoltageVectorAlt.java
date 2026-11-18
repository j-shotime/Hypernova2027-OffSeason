package main.java.first.robot.subsystems.drivetrain;

import first.robot.subsystems.drivetrain.VoltageVector;

public class VoltageVectorAlt {
    public VoltageVectorAlt(Voltage voltage, Angle angle) {
        // Translation2d(distance, Rotation2d) automatically handles negative magnitudes
        // by flipping the vector 180 degrees, and Rotation2d wraps angles cleanly.
        this(new Translation2d(voltage.in(Units.Volts), new Rotation2d(angle)));
    }

    public Voltage voltage() {
        return Units.Volts.of(vec.getNorm());
    }

    public Angle angle() {
        // MathUtil.angleModulus wraps to [-pi, pi], or use inputModulus for [0, 2*pi]
        return Units.Radians.of(MathUtil.inputModulus(vec.getAngle().getRadians(), 0.0, 2.0 * Math.PI));
    }
    
    public Voltage x() {
        return Units.Volts.of(vec.getX());
    }

    public Voltage y() {
        return Units.Volts.of(vec.getY());
    }

    // Vector addition is now a single built-in call
    public VoltageVector addVector(VoltageVector other) {
        return new VoltageVector(this.vec.plus(other.vec));
    }

    // Clamping magnitude while preserving angle
    public static VoltageVector Max(VoltageVector voltageVector, Voltage maxVolts) {
        if (voltageVector.voltage().gt(maxVolts)) {
            return new VoltageVector(maxVolts, voltageVector.angle());
        }
        return voltageVector;
    }
}
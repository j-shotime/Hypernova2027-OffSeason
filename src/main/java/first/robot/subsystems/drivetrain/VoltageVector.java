package first.robot.subsystems.drivetrain;

import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Voltage;

import pabeles.concurrency.IntOperatorTask.Max;

import static org.wpilib.units.Units.Volt;

import javax.xml.crypto.URIReferenceException;

import org.wpilib.driverstation.GenericHID.RumbleType;
import org.wpilib.units.Unit;
import org.wpilib.units.Units;
import org.wpilib.units.VoltageUnit;

public record VoltageVector(
    Voltage voltage,
    Angle angle
) 
{
    
    public VoltageVector(Voltage voltage, Angle angle) 
    {
        
        if(voltage.in(Units.Volts)<0)
        {
            this.voltage = voltage.times(-1);
            this.angle = normalize(angle.plus(Units.Radians.of(Math.PI)));
            return;
        }
        this.voltage = voltage;
        this.angle = normalize(angle);
    }

    public VoltageVector addVector(VoltageVector other) {
        Voltage newVoltage = Units.Volts.of(Math.hypot(this.x().plus(other.x()).in(Units.Volts), this.y().plus(other.y()).in(Units.Volts)));
        Angle newAngle = Units.Radians.of(Math.atan2(this.y().plus(other.y()).in(Units.Volts), this.x().plus(other.x()).in(Units.Volts)));
        return new VoltageVector(newVoltage, newAngle);
    }

    public static VoltageVector Max(VoltageVector voltageVector, Voltage maxVolts)
    {
        double output = Math.clamp(voltageVector.voltage.in(Units.Volts),-maxVolts.in(Units.Volts),maxVolts.in(Units.Volts));
        return new VoltageVector(Units.Volts.of(output), voltageVector.angle);
    }

    private static Angle normalize(Angle angle) {
        double radians = angle.in(Units.Radians);

        radians %= 2.0 * Math.PI;

        if (radians < 0) {
            radians += 2.0 * Math.PI;
        }

        return Units.Radians.of(radians);
    }

    public Voltage x() 
    {
        return voltage.times(Math.cos(angle.in(Units.Radians)));
    }

    public Voltage y() 
    {
        return voltage.times(Math.sin(angle.in(Units.Radians)));
    }
}
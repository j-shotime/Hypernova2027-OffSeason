package first.robot.subsystems.drivetrain;

import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.LinearVelocity;
import org.wpilib.units.Units;

public record VelocityVector(
    LinearVelocity velocity,
    Angle angle
) 
{
    public VelocityVector(LinearVelocity velocity, Angle angle) 
    {
        this.velocity = velocity;
        this.angle = normalize(angle);
    }

    private static Angle normalize(Angle angle) {
        double radians = angle.in(Units.Radians);

        radians %= 2 * Math.PI;

        if (radians < 0) {
            radians += 2 * Math.PI;
        }

        return Units.Radians.of(radians);
    }

    public LinearVelocity x() 
    {
        return velocity.times(Math.cos(angle.in(Units.Radians)));
    }

    public LinearVelocity y() 
    {
        return velocity.times(Math.sin(angle.in(Units.Radians)));
    }
}
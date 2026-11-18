package main.java.first.robot.subsystems.drivetrain;

public record VelocityVectorAlt(
    LinearVelocity velocity,
    Angle angle
) 
{
    public VelocityVectorAlt(LinearVelocity velocity, Angle angle) 
    {
          this(new Translation2d(velocity.in(Units.MetersPerSecond), new Rotation2d(angle)));
    }

      // Getters matching your original record components
    public LinearVelocity velocity() {
        return Units.MetersPerSecond.of(vec.getNorm());
    }

    public Angle angle() {
        return Units.Radians.of(MathUtil.inputModulus(vec.getAngle().getRadians(), 0.0, 2.0 * Math.PI));
    }

    public LinearVelocity x() {
        return Units.MetersPerSecond.of(vec.getX());
    }

    public LinearVelocity y() {
        return Units.MetersPerSecond.of(vec.getY());
    }
}

package first.robot.subsystems.drivetrain;

import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.Volts;

import org.opencv.core.Mat;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.units.Unit;
import org.wpilib.units.Units;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.LinearVelocity;
import org.wpilib.units.measure.Voltage;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class SwerveModule {
    public record SteerCommand(double targetRadians, double driveVoltage) {}

    private final TalonFX driveMotor;
    private final TalonFX steerMotor;

    public SwerveModule(TalonFX driveMotor, TalonFX steerMotor) {
        this.driveMotor = driveMotor;
        this.steerMotor = steerMotor;
    }
    public SwerveModule(int driveMotorId, int steerMotorId) {
        CANBus CAN_S0 = new CANBus(CANPort.CAN_S0);
        this(new TalonFX(driveMotorId, CAN_S0), new TalonFX(steerMotorId, CAN_S0));
    }

    public SwerveModule(int driveMotorId, int steerMotorId, CANBus bus) {
        this(new TalonFX(driveMotorId, bus), new TalonFX(steerMotorId, bus));
    }

    public SwerveModule(int driveMotorId, CANBus driveBus, int steerMotorId, CANBus steerBus) {
        this(new TalonFX(driveMotorId, driveBus), new TalonFX(steerMotorId, steerBus));
    }

    public static double wrapAngle(double angle) {
        return ((angle % Math.TAU) + Math.TAU) % Math.TAU;
    }
    private static double wrapHalfAngle(double angle) {
        return ((angle % Math.PI) + Math.PI) % Math.PI;
    }

    public static double shortestAngleDelta(double current, double target)
    {
        return Math.atan2(Math.sin(target-current), Math.cos(target-current));
    }

    public static SteerCommand calculateSteerCommand(double currentRadians, double targetRadians, double driveVoltage) {
        double wrappedCurrent = wrapAngle(currentRadians);
        double wrappedTarget = wrapAngle(targetRadians);
        double shortestDelta = shortestAngleDelta(wrappedCurrent, wrappedTarget);

        shortestDelta = wrapAngle(shortestDelta);

        double outputTargetRadians = wrappedTarget;
        double outputDriveVoltage = driveVoltage;

        if (Math.abs(shortestDelta) > Math.PI / 2.0) {
            outputTargetRadians = wrapAngle(wrappedTarget + Math.PI);
            outputDriveVoltage = -driveVoltage;
        }

        return new SteerCommand(outputTargetRadians, outputDriveVoltage);
    }

    public void set(VoltageVector voltageVector) {
        Voltage driveVoltage = voltageVector.voltage();

        double targetRadians = wrapAngle(
            voltageVector.angle().in(Radians)
        );

        double currentRadians = wrapAngle(
            steerMotor.getPosition().getValue().in(Radians)
        );

        SteerCommand steerCommand = calculateSteerCommand(currentRadians, targetRadians, driveVoltage.in(Units.Volts));

        //Angle steerAngle = Radians.of(steerCommand.targetRadians());
        Angle steerAngle;
        if(driveVoltage.in(Volts) == 0) steerAngle = Units.Radians.of(currentRadians);
        else steerAngle = Radians.of(steerCommand.targetRadians());

        driveMotor.setVoltage(Units.Volts.of(steerCommand.driveVoltage()));

        if (voltageVector.voltage().in(Units.Volts) == 0) {
            return;
        }

        steerMotor.setControl(new PositionVoltage(steerAngle));
    }
    public void set(VelocityVector velocityVector) {
        throw new UnsupportedOperationException("Velocity control is not implemented yet.");
    }

    /**
     * Returns the steer encoder angle in radians.
     */
    public double getSteerAngleRadians() {
        return steerMotor.getPosition().getValue().in(Units.Radians);
    }

    public void resetEncoders() {
        if (driveMotor != null) {
            driveMotor.setPosition(0);
        }
        if (steerMotor != null) {
            steerMotor.setPosition(0);
        }
    }
}

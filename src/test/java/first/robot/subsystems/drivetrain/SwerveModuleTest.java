package first.robot.subsystems.drivetrain;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class SwerveModuleTest {
    @Test
    void flipsWheelAndReversesDriveWhenCrossingHalfRotation() {
        var command = SwerveModule.calculateSteerCommand(0.0, Math.PI, 6.0);

        assertEquals(0.0, command.targetRadians(), 1e-9);
        assertEquals(-6.0, command.driveVoltage(), 1e-9);
    }

    @Test
    void keepsForwardDriveWhenCloseToTarget() {
        var command = SwerveModule.calculateSteerCommand(0.0, 0.2, 6.0);

        assertEquals(0.2, command.targetRadians(), 1e-9);
        assertEquals(6.0, command.driveVoltage(), 1e-9);
    }
}

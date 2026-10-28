package first.robot.subsystems.drivetrain;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Volts;

import org.wpilib.command2.SubsystemBase;
import org.wpilib.units.Unit;
import org.wpilib.units.Units;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Voltage;

import com.ctre.phoenix6.CANBus;

import pabeles.concurrency.IntOperatorTask.Max;

public class SwerveDrive extends SubsystemBase {
    private static final double TRANSLATION_ANGLE_HOLD_THRESHOLD = 0.08;
    private final double widthInches;
    private final double heightInches;
    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;
    private Angle rotationAngle;

    public SwerveDrive(double widthInches, double heightInches, SwerveModule frontLeft, SwerveModule frontRight, SwerveModule backLeft, SwerveModule backRight) {
        this.widthInches = widthInches;
        this.heightInches = heightInches;
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        rotationAngle = Units.Radians.of(Math.atan2(widthInches, heightInches)+(Math.PI/2));
    }

    public SwerveDrive(
        double widthInches,
        double heightInches,
        int frontLeftDriveId, int frontLeftSteerId,
        int frontRightDriveId, int frontRightSteerId,
        int backLeftDriveId, int backLeftSteerId,
        int backRightDriveId, int backRightSteerId
    ) {
        this(
            widthInches,
            heightInches,
            new SwerveModule(frontLeftDriveId, frontLeftSteerId),
            new SwerveModule(frontRightDriveId, frontRightSteerId),
            new SwerveModule(backLeftDriveId, backLeftSteerId),
            new SwerveModule(backRightDriveId, backRightSteerId)
        );
    }

    public SwerveDrive(
        double widthInches,
        double heightInches,
        int frontLeftDriveId, CANBus frontLeftDriveBus, int frontLeftSteerId, CANBus frontLeftSteerBus,
        int frontRightDriveId, CANBus frontRightDriveBus, int frontRightSteerId, CANBus frontRightSteerBus,
        int backLeftDriveId, CANBus backLeftDriveBus, int backLeftSteerId, CANBus backLeftSteerBus,
        int backRightDriveId, CANBus backRightDriveBus, int backRightSteerId, CANBus backRightSteerBus
    ) {
        this(
            widthInches,
            heightInches,
            new SwerveModule(frontLeftDriveId, frontLeftDriveBus, frontLeftSteerId, frontLeftSteerBus),
            new SwerveModule(frontRightDriveId, frontRightDriveBus, frontRightSteerId, frontRightSteerBus),
            new SwerveModule(backLeftDriveId, backLeftDriveBus, backLeftSteerId, backLeftSteerBus),
            new SwerveModule(backRightDriveId, backRightDriveBus, backRightSteerId, backRightSteerBus)
        );
    }

    public double widthInches() {
        return widthInches;
    }

    public double heightInches() {
        return heightInches;
    }

    public SwerveModule frontLeft() {
        return frontLeft;
    }

    public SwerveModule frontRight() {
        return frontRight;
    }

    public SwerveModule backLeft() {
        return backLeft;
    }

    public SwerveModule backRight() {
        return backRight;
    }

    public void setModuleVelocities(VelocityVector frontLeftVelocity, VelocityVector frontRightVelocity, VelocityVector backLeftVelocity, VelocityVector backRightVelocity) {
        frontLeft.set(frontLeftVelocity);
        frontRight.set(frontRightVelocity);
        backLeft.set(backLeftVelocity);
        backRight.set(backRightVelocity);
    }

    public void setModuleVoltages(VoltageVector frontLeftVoltage, VoltageVector frontRightVoltage, VoltageVector backLeftVoltage, VoltageVector backRightVoltage) {
        frontLeft.set(frontLeftVoltage);
        frontRight.set(frontRightVoltage);
        backLeft.set(backLeftVoltage);
        backRight.set(backRightVoltage);
    }

    public void stop() {
        frontLeft.set(new VoltageVector(Units.Volts.zero(), Units.Radians.zero()));
        frontRight.set(new VoltageVector(Units.Volts.zero(), Units.Radians.zero()));
        backLeft.set(new VoltageVector(Units.Volts.zero(), Units.Radians.zero()));
        backRight.set(new VoltageVector(Units.Volts.zero(), Units.Radians.zero()));
    }

    public void setDriveVoltage(Voltage voltage) {
        stop();
        frontLeft.set(new VoltageVector(voltage, Units.Radians.zero()));
        frontRight.set(new VoltageVector(voltage, Units.Radians.zero()));
        backLeft.set(new VoltageVector(voltage, Units.Radians.zero()));
        backRight.set(new VoltageVector(voltage, Units.Radians.zero()));
    }

    // public void localArcadeDrive(double forward, double strafe, double rotation, double maxVoltageVolts) {        
        
    //     if (forward == 0 && strafe == 0 && rotation == 0) {
    //         stop();
    //         return;
    //     }

    //     double translationMagnitude = Math.hypot(forward, strafe);
        
    //     Angle translationAngle = Units.Radians.of(Math.atan2(strafe, forward));

    //     VoltageVector DriveVector = new VoltageVector(Units.Volts.of(translationMagnitude * maxVoltageVolts / 2), translationAngle);

    //     // Get rotation vector components
    //     double rotX = rotation * Math.cos(rotationAngle.in(Units.Radians));
    //     double rotY = rotation * Math.sin(rotationAngle.in(Units.Radians));

    //     // Module 1 (frontLeft): horizontal flip (negate x)
    //     Angle angle1 = Units.Radians.of(Math.atan2(rotY, -rotX));
    //     // Module 2 (frontRight): normal
    //     Angle angle2 = rotationAngle;
    //     // Module 3 (backLeft): normal (180 offset)
    //     Angle angle3 = rotationAngle.plus(Degrees.of(180));
    //     // Module 4 (backRight): vertical flip (negate y)
    //     Angle angle4 = Units.Radians.of(Math.atan2(-rotY, rotX));

    //     setModuleVoltages(
    //         DriveVector.addVector(new VoltageVector(Units.Volts.of(Math.abs(rotation)*maxVoltageVolts/2), angle1)),
    //         DriveVector.addVector(new VoltageVector(Units.Volts.of(rotation*maxVoltageVolts/2), angle2)),
    //         DriveVector.addVector(new VoltageVector(Units.Volts.of(rotation*maxVoltageVolts/2), angle3)),
    //         DriveVector.addVector(new VoltageVector(Units.Volts.of(Math.abs(rotation)*maxVoltageVolts/2), angle4))
    //     );
    // }
    public void localArcadeDrive(double forward, double strafe, double rotation, double maxVoltageVolts) {        
        
        if (forward == 0 && strafe == 0 && rotation == 0) {
            stop();
            return;
        }

        double translationMagnitude = Math.hypot(forward, strafe);
        
        Angle translationAngle = Units.Radians.of(Math.atan2(strafe, forward));

        VoltageVector DriveVector = new VoltageVector(Units.Volts.of(translationMagnitude * maxVoltageVolts), translationAngle);

        // Get rotation vector components
        double rotX = rotation * maxVoltageVolts * Math.cos(rotationAngle.in(Units.Radians));
        double rotY = rotation * maxVoltageVolts * Math.sin(rotationAngle.in(Units.Radians));

        // Module 1 (frontLeft): horizontal flip (negate x)
        Angle angle1 = Units.Radians.of(Math.atan2(rotY, -rotX));
        // Module 2 (frontRight): normal
        Angle angle2 = rotationAngle;
        // Module 3 (backLeft): normal (180 offset)
        Angle angle3 = rotationAngle.plus(Degrees.of(180));
        // Module 4 (backRight): vertical flip (negate y)
        Angle angle4 = Units.Radians.of(Math.atan2(-rotY, rotX));

        VoltageVector frontLeft = DriveVector.addVector(new VoltageVector(Units.Volts.of(Math.abs(rotation)*maxVoltageVolts), angle1));
        VoltageVector frontRight = DriveVector.addVector(new VoltageVector(Units.Volts.of(rotation*maxVoltageVolts), angle2));
        VoltageVector backLeft = DriveVector.addVector(new VoltageVector(Units.Volts.of(rotation*maxVoltageVolts), angle3));
        VoltageVector backRight = DriveVector.addVector(new VoltageVector(Units.Volts.of(Math.abs(rotation)*maxVoltageVolts), angle4));

        double max = Math.max(Math.abs(frontLeft.voltage().in(Volts)), 
                     Math.max(Math.abs(frontRight.voltage().in(Volts)),
                     Math.max(Math.abs(backLeft.voltage().in(Volts)),
                              Math.abs(backRight.voltage().in(Volts)))));
        max /= maxVoltageVolts;
        if(max > 1)
        {
            frontLeft = new VoltageVector(Units.Volts.of(frontLeft.voltage().in(Volts)/max), frontLeft.angle());
            frontRight = new VoltageVector(Units.Volts.of(frontRight.voltage().in(Volts)/max), frontRight.angle());
            backLeft = new VoltageVector(Units.Volts.of(backLeft.voltage().in(Volts)/max), backLeft.angle());
            backRight = new VoltageVector(Units.Volts.of(backRight.voltage().in(Volts)/max), backRight.angle());
        }
        setModuleVoltages(frontLeft, frontRight, backLeft, backRight);
    }

    public void localArcadeDrive(double forward, double strafe, double rotation) {
        localArcadeDrive(forward, strafe, rotation, 12.0);
    }

    /**
     * Field-centric (global) arcade drive.
     *
     * @param forward forward command in field coordinates
     * @param strafe  strafe command in field coordinates
     * @param rotation rotation command (robot-relative)
     * @param maxVoltageVolts max voltage
     * @param yawRadians current robot yaw in radians (positive CCW)
     */
    public void globalArcadeDrive(double forward, double strafe, double rotation, double maxVoltageVolts, double yawRadians) {
        // Rotate the field vector into robot-relative coordinates by -yaw:
        double cos = Math.cos(yawRadians);
        double sin = Math.sin(yawRadians);

        double robotForward = cos * forward + sin * strafe;
        double robotStrafe = -sin * forward + cos * strafe;

        localArcadeDrive(robotForward, robotStrafe, rotation, maxVoltageVolts);
    }

    public void globalArcadeDrive(double forward, double strafe, double rotation, double yawRadians) {
        globalArcadeDrive(forward, strafe, rotation, 12.0, yawRadians);
    }

    public void resetEncoders() {
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        backLeft.resetEncoders();
        backRight.resetEncoders();
    }

    
}

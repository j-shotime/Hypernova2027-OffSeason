package first.robot.custom;

import org.wpilib.command2.button.CommandGenericHID;
import org.wpilib.driverstation.Gamepad;
import org.wpilib.driverstation.GenericHID;
import org.wpilib.driverstation.GenericHID.RumbleType;

public class VexController {
    private final CommandGenericHID hid;
    private static final double AXIS_DEADBAND = 0.05;

    // Constructor
    public VexController(int port) {
        this.hid = new CommandGenericHID(port);
    }

    public VexController(CommandGenericHID hid) {
        this.hid = hid;
    }

    // Getters for axes (left stick, right stick, triggers, etc.)
    public double getLeftX() {
        return driveCurve(hid.getRawAxis(0));
    }

    public double getLeftY() {
        return -driveCurve(hid.getRawAxis(1));
    }

    public double getRightX() {
        return driveCurve(hid.getRawAxis(2));
    }

    public double getRightY() {
        return -driveCurve(hid.getRawAxis(3));
    }

    // Button getters - Face buttons
    public boolean getButtonA() {
        return hid.button(1).getAsBoolean();
    }

    public boolean getButtonB() {
        return hid.button(0).getAsBoolean();
    }

    public boolean getButtonX() {
        return hid.button(3).getAsBoolean();
    }

    public boolean getButtonY() {
        return hid.button(2).getAsBoolean();
    }

    // Shoulder buttons
    public boolean getButtonL1() {
        return hid.button(4).getAsBoolean();
    }

    public boolean getButtonR1() {
        return hid.button(5).getAsBoolean();
    }

    public boolean getButtonL2() {
        return hid.button(6).getAsBoolean();
    }

    public boolean getButtonR2() {
        return hid.button(7).getAsBoolean();
    }

    // Directional buttons
    public boolean getButtonUp() {
        return hid.button(12).getAsBoolean();
    }

    public boolean getButtonDown() {
        return hid.button(13).getAsBoolean();
    }

    public boolean getButtonLeft() {
        return hid.button(14).getAsBoolean();
    }

    public boolean getButtonRight() {
        return hid.button(15).getAsBoolean();
    }

    public boolean getButtonCenter() {
        return hid.button(13).getAsBoolean();
    }

    public void testPrintAllValues() {
        System.out.println("=== VEXCONTROLLER PORT 0 DEBUG ===");
        System.out.println("Axes - LeftX: " + getLeftX() + " LeftY: " + getLeftY() + " RightX: " + getRightX() + " RightY: " + getRightY());
        for (int axis = 0; axis < 4; axis++) {
            System.out.println("  Raw Axis " + axis + ": " + hid.getRawAxis(axis));
        }
        System.out.println("Buttons:");
        for (int button = 1; button <= 16; button++) {
            if (hid.button(button).getAsBoolean()) {
                System.out.println("  Button " + button + " PRESSED");
            }
        }
    }

    // Utility
    public CommandGenericHID getHID() {
        return hid;
    }

    public boolean isConnected() {
        return hid.isConnected();
    }

    public void setRumble(RumbleType type, double value) {
        hid.setRumble(type, value);
    }

    private static double driveCurve(double value) {
        if (Math.abs(value) < AXIS_DEADBAND) {
            return 0.0;
        }

        double scaledValue = (Math.abs(value) - AXIS_DEADBAND) / (1.0 - AXIS_DEADBAND);
        double curvedValue = Math.pow(scaledValue,2);
        return Math.copySign(curvedValue, value);
    }
}

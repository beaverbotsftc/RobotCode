package org.firstinspires.ftc.teamcode.subsystems;

import org.beaverbots.BeaverCommand.Subsystem;

public class Gamepad implements Subsystem {
    private com.qualcomm.robotcore.hardware.Gamepad gamepad;

    // Joystick values (No edge detection as requested)
    private double leftX = 0;
    private double leftY = 0;
    private double rightX = 0;
    private double rightY = 0;

    // Current Values
    private double leftTrigger = 0;
    private double rightTrigger = 0;
    private boolean leftBumper = false;
    private boolean rightBumper = false;
    private boolean circle = false;
    private boolean cross = false;
    private boolean square = false;
    private boolean triangle = false;

    // Previous State Tracking (for edge detection)
    private boolean lastLeftTriggerState = false;
    private boolean lastRightTriggerState = false;
    private boolean lastLeftBumper = false;
    private boolean lastRightBumper = false;
    private boolean lastCircle = false;
    private boolean lastCross = false;
    private boolean lastSquare = false;
    private boolean lastTriangle = false;

    // Edge Detection Results
    private boolean leftTriggerJustPressed = false;
    private boolean leftTriggerJustReleased = false;
    private boolean rightTriggerJustPressed = false;
    private boolean rightTriggerJustReleased = false;

    private boolean leftBumperJustPressed = false;
    private boolean leftBumperJustReleased = false;
    private boolean rightBumperJustPressed = false;
    private boolean rightBumperJustReleased = false;

    private boolean circleJustPressed = false;
    private boolean circleJustReleased = false;
    private boolean crossJustPressed = false;
    private boolean crossJustReleased = false;
    private boolean squareJustPressed = false;
    private boolean squareJustReleased = false;
    private boolean triangleJustPressed = false;
    private boolean triangleJustReleased = false;

    public Gamepad(com.qualcomm.robotcore.hardware.Gamepad gamepad) {
        gamepad.setLedColor(1 / 256.0, 0.0, 64 / 256.0, com.qualcomm.robotcore.hardware.Gamepad.LED_DURATION_CONTINUOUS);
        this.gamepad = gamepad;
    }

    @Override
    public void periodic() {
        leftX = gamepad.left_stick_x;
        leftY = -gamepad.left_stick_y; // Inverted Y
        rightX = gamepad.right_stick_x;
        rightY = -gamepad.right_stick_y; // Inverted Y

        leftTrigger = gamepad.left_trigger;
        rightTrigger = gamepad.right_trigger;
        leftBumper = gamepad.left_bumper;
        rightBumper = gamepad.right_bumper;
        circle = gamepad.circle;
        cross = gamepad.cross;
        square = gamepad.square;
        triangle = gamepad.triangle;

        // --- Triggers ---
        // Logic: > 0 is considered pressed
        boolean currentLeftTriggerState = leftTrigger > 0;
        leftTriggerJustPressed = currentLeftTriggerState && !lastLeftTriggerState;
        leftTriggerJustReleased = !currentLeftTriggerState && lastLeftTriggerState;
        lastLeftTriggerState = currentLeftTriggerState;

        boolean currentRightTriggerState = rightTrigger > 0;
        rightTriggerJustPressed = currentRightTriggerState && !lastRightTriggerState;
        rightTriggerJustReleased = !currentRightTriggerState && lastRightTriggerState;
        lastRightTriggerState = currentRightTriggerState;

        // --- Bumpers ---
        leftBumperJustPressed = leftBumper && !lastLeftBumper;
        leftBumperJustReleased = !leftBumper && lastLeftBumper;
        lastLeftBumper = leftBumper;

        rightBumperJustPressed = rightBumper && !lastRightBumper;
        rightBumperJustReleased = !rightBumper && lastRightBumper;
        lastRightBumper = rightBumper;

        // --- Face Buttons ---
        circleJustPressed = circle && !lastCircle;
        circleJustReleased = !circle && lastCircle;
        lastCircle = circle;

        crossJustPressed = cross && !lastCross;
        crossJustReleased = !cross && lastCross;
        lastCross = cross;

        squareJustPressed = square && !lastSquare;
        squareJustReleased = !square && lastSquare;
        lastSquare = square;

        triangleJustPressed = triangle && !lastTriangle;
        triangleJustReleased = !triangle && lastTriangle;
        lastTriangle = triangle;
    }

    // --- Standard Getters ---

    public double getLeftX() { return leftX; }
    public double getLeftY() { return leftY; }
    public double getRightX() { return rightX; }
    public double getRightY() { return rightY; }

    public double getLeftTrigger() { return leftTrigger; }
    public double getRightTrigger() { return rightTrigger; }

    public boolean getLeftBumper() { return leftBumper; }
    public boolean getRightBumper() { return rightBumper; }

    public boolean getCircle() { return circle; }
    public boolean getCross() { return cross; }
    public boolean getSquare() { return square; }
    public boolean getTriangle() { return triangle; }

    // --- Just Pressed (Rising Edge) Getters ---

    public boolean getLeftTriggerJustPressed() { return leftTriggerJustPressed; }
    public boolean getRightTriggerJustPressed() { return rightTriggerJustPressed; }

    public boolean getLeftBumperJustPressed() { return leftBumperJustPressed; }
    public boolean getRightBumperJustPressed() { return rightBumperJustPressed; }

    public boolean getCircleJustPressed() { return circleJustPressed; }
    public boolean getCrossJustPressed() { return crossJustPressed; }
    public boolean getSquareJustPressed() { return squareJustPressed; }
    public boolean getTriangleJustPressed() { return triangleJustPressed; }

    // --- Just Released (Falling Edge) Getters ---

    public boolean getLeftTriggerJustReleased() { return leftTriggerJustReleased; }
    public boolean getRightTriggerJustReleased() { return rightTriggerJustReleased; }

    public boolean getLeftBumperJustReleased() { return leftBumperJustReleased; }
    public boolean getRightBumperJustReleased() { return rightBumperJustReleased; }

    public boolean getCircleJustReleased() { return circleJustReleased; }
    public boolean getCrossJustReleased() { return crossJustReleased; }
    public boolean getSquareJustReleased() { return squareJustReleased; }
    public boolean getTriangleJustReleased() { return triangleJustReleased; }
}
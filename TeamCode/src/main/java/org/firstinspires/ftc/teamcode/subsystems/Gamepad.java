package org.firstinspires.ftc.teamcode.subsystems;

import org.beaverbots.beaver.command.Subsystem;

public class Gamepad implements Subsystem {

    private com.qualcomm.robotcore.hardware.Gamepad gamepad;

    private static final double DEADZONE = 0.1;

    // --- Joystick Values ---
    private double leftX = 0;
    private double leftY = 0;
    private double rightX = 0;
    private double rightY = 0;

    // --- Current Button Values ---
    private double leftTrigger = 0;
    private double rightTrigger = 0;
    private boolean leftBumper = false;
    private boolean rightBumper = false;
    private boolean circle = false;
    private boolean cross = false;
    private boolean square = false;
    private boolean triangle = false;
    private boolean guide = false;

    private boolean dpadUp = false;
    private boolean dpadDown = false;
    private boolean dpadLeft = false;
    private boolean dpadRight = false;

    // Stick buttons
    private boolean leftStickPressed = false;
    private boolean rightStickPressed = false;

    // --- Previous State Tracking ---
    private boolean lastLeftTriggerState = false;
    private boolean lastRightTriggerState = false;
    private boolean lastLeftBumper = false;
    private boolean lastRightBumper = false;
    private boolean lastCircle = false;
    private boolean lastCross = false;
    private boolean lastSquare = false;
    private boolean lastTriangle = false;
    private boolean lastGuide = false;

    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;

    private boolean lastLeftStickPressed = false;
    private boolean lastRightStickPressed = false;

    // --- Edge Detection Output ---
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
    private boolean guideJustPressed = false;
    private boolean guideJustReleased = false;

    private boolean dpadUpJustPressed = false;
    private boolean dpadUpJustReleased = false;
    private boolean dpadDownJustPressed = false;
    private boolean dpadDownJustReleased = false;
    private boolean dpadLeftJustPressed = false;
    private boolean dpadLeftJustReleased = false;
    private boolean dpadRightJustPressed = false;
    private boolean dpadRightJustReleased = false;

    private boolean leftStickJustPressed = false;
    private boolean leftStickJustReleased = false;
    private boolean rightStickJustPressed = false;
    private boolean rightStickJustReleased = false;

    public Gamepad(com.qualcomm.robotcore.hardware.Gamepad gamepad) {
        gamepad.setLedColor(
                1.0 / 256.0,
                0.0,
                64.0 / 256.0,
                com.qualcomm.robotcore.hardware.Gamepad.LED_DURATION_CONTINUOUS);
        this.gamepad = gamepad;
    }

    private double applyDeadzone(double value) {
        return Math.abs(value) < DEADZONE ? 0 : value;
    }

    @Override
    public void periodic() {

        // --- Joysticks ---
        leftX = applyDeadzone(gamepad.left_stick_x);
        leftY = applyDeadzone(-gamepad.left_stick_y);
        rightX = applyDeadzone(gamepad.right_stick_x);
        rightY = applyDeadzone(-gamepad.right_stick_y);

        // --- Triggers & Buttons ---
        leftTrigger = applyDeadzone(gamepad.left_trigger);
        rightTrigger = applyDeadzone(gamepad.right_trigger);

        leftBumper = gamepad.left_bumper;
        rightBumper = gamepad.right_bumper;
        circle = gamepad.circle;
        cross = gamepad.cross;
        square = gamepad.square;
        triangle = gamepad.triangle;
        guide = gamepad.guide;

        // --- D-Pad ---
        dpadUp = gamepad.dpad_up;
        dpadDown = gamepad.dpad_down;
        dpadLeft = gamepad.dpad_left;
        dpadRight = gamepad.dpad_right;

        // --- Stick Buttons ---
        leftStickPressed = gamepad.left_stick_button;
        rightStickPressed = gamepad.right_stick_button;

        // ================================
        // =     EDGE DETECTION LOGIC     =
        // ================================

        // --- Trigger Edges ---
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

        guideJustPressed = guide && !lastGuide;
        guideJustReleased = !guide && lastGuide;
        lastGuide = guide;

        // --- D-Pad ---
        dpadUpJustPressed = dpadUp && !lastDpadUp;
        dpadUpJustReleased = !dpadUp && lastDpadUp;
        lastDpadUp = dpadUp;

        dpadDownJustPressed = dpadDown && !lastDpadDown;
        dpadDownJustReleased = !dpadDown && lastDpadDown;
        lastDpadDown = dpadDown;

        dpadLeftJustPressed = dpadLeft && !lastDpadLeft;
        dpadLeftJustReleased = !dpadLeft && lastDpadLeft;
        lastDpadLeft = dpadLeft;

        dpadRightJustPressed = dpadRight && !lastDpadRight;
        dpadRightJustReleased = !dpadRight && lastDpadRight;
        lastDpadRight = dpadRight;

        // --- Stick Buttons ---
        leftStickJustPressed = leftStickPressed && !lastLeftStickPressed;
        leftStickJustReleased = !leftStickPressed && lastLeftStickPressed;
        lastLeftStickPressed = leftStickPressed;

        rightStickJustPressed = rightStickPressed && !lastRightStickPressed;
        rightStickJustReleased = !rightStickPressed && lastRightStickPressed;
        lastRightStickPressed = rightStickPressed;
    }

    // ================================
    // =           RUMBLE             =
    // ================================

    public void rumble() {
        gamepad.rumble(com.qualcomm.robotcore.hardware.Gamepad.RUMBLE_DURATION_CONTINUOUS);
    }

    public void rumble(int duration) {
        gamepad.rumble(duration);
    }

    public void rumble(double rumble) {
        gamepad.rumble(rumble, rumble,
                com.qualcomm.robotcore.hardware.Gamepad.RUMBLE_DURATION_CONTINUOUS);
    }

    public void rumble(double rumble, int duration) {
        gamepad.rumble(rumble, rumble, duration);
    }

    public void rumble(double leftRumble, double rightRumble) {
        gamepad.rumble(leftRumble, rightRumble,
                com.qualcomm.robotcore.hardware.Gamepad.RUMBLE_DURATION_CONTINUOUS);
    }

    public void rumble(double leftRumble, double rightRumble, int duration) {
        gamepad.rumble(leftRumble, rightRumble, duration);
    }

    public void stopRumble() {
        gamepad.stopRumble();
    }

    // ================================
    // =        STANDARD GETTERS       =
    // ================================

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
    public boolean getGuide() { return guide; }

    public boolean getDpadUp() { return dpadUp; }
    public boolean getDpadDown() { return dpadDown; }
    public boolean getDpadLeft() { return dpadLeft; }
    public boolean getDpadRight() { return dpadRight; }

    public boolean getLeftStickPressed() { return leftStickPressed; }
    public boolean getRightStickPressed() { return rightStickPressed; }

    // ================================
    // =        JUST PRESSED GETTERS   =
    // ================================

    public boolean getLeftTriggerJustPressed() { return leftTriggerJustPressed; }
    public boolean getRightTriggerJustPressed() { return rightTriggerJustPressed; }

    public boolean getLeftBumperJustPressed() { return leftBumperJustPressed; }
    public boolean getRightBumperJustPressed() { return rightBumperJustPressed; }

    public boolean getCircleJustPressed() { return circleJustPressed; }
    public boolean getCrossJustPressed() { return crossJustPressed; }
    public boolean getSquareJustPressed() { return squareJustPressed; }
    public boolean getTriangleJustPressed() { return triangleJustPressed; }
    public boolean getGuideJustPressed() { return guideJustPressed; }

    public boolean getDpadUpJustPressed() { return dpadUpJustPressed; }
    public boolean getDpadDownJustPressed() { return dpadDownJustPressed; }
    public boolean getDpadLeftJustPressed() { return dpadLeftJustPressed; }
    public boolean getDpadRightJustPressed() { return dpadRightJustPressed; }

    public boolean getLeftStickJustPressed() { return leftStickJustPressed; }
    public boolean getRightStickJustPressed() { return rightStickJustPressed; }

    // ================================
    // =       JUST RELEASED GETTERS   =
    // ================================

    public boolean getLeftTriggerJustReleased() { return leftTriggerJustReleased; }
    public boolean getRightTriggerJustReleased() { return rightTriggerJustReleased; }

    public boolean getLeftBumperJustReleased() { return leftBumperJustReleased; }
    public boolean getRightBumperJustReleased() { return rightBumperJustReleased; }

    public boolean getCircleJustReleased() { return circleJustReleased; }
    public boolean getCrossJustReleased() { return crossJustReleased; }
    public boolean getSquareJustReleased() { return squareJustReleased; }
    public boolean getTriangleJustReleased() { return triangleJustReleased; }
    public boolean getGuideJustReleased() { return guideJustReleased; }

    public boolean getDpadUpJustReleased() { return dpadUpJustReleased; }
    public boolean getDpadDownJustReleased() { return dpadDownJustReleased; }
    public boolean getDpadLeftJustReleased() { return dpadLeftJustReleased; }
    public boolean getDpadRightJustReleased() { return dpadRightJustReleased; }

    public boolean getLeftStickJustReleased() { return leftStickJustReleased; }
    public boolean getRightStickJustReleased() { return rightStickJustReleased; }
}

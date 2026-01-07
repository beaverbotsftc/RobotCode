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

    // --- Toggle States ---
    private boolean leftTriggerPressedToggle = false;
    private boolean leftTriggerReleasedToggle = false;
    private boolean rightTriggerPressedToggle = false;
    private boolean rightTriggerReleasedToggle = false;

    private boolean leftBumperPressedToggle = false;
    private boolean leftBumperReleasedToggle = false;
    private boolean rightBumperPressedToggle = false;
    private boolean rightBumperReleasedToggle = false;

    private boolean circlePressedToggle = false;
    private boolean circleReleasedToggle = false;
    private boolean crossPressedToggle = false;
    private boolean crossReleasedToggle = false;
    private boolean squarePressedToggle = false;
    private boolean squareReleasedToggle = false;
    private boolean trianglePressedToggle = false;
    private boolean triangleReleasedToggle = false;
    private boolean guidePressedToggle = false;
    private boolean guideReleasedToggle = false;

    private boolean dpadUpPressedToggle = false;
    private boolean dpadUpReleasedToggle = false;
    private boolean dpadDownPressedToggle = false;
    private boolean dpadDownReleasedToggle = false;
    private boolean dpadLeftPressedToggle = false;
    private boolean dpadLeftReleasedToggle = false;
    private boolean dpadRightPressedToggle = false;
    private boolean dpadRightReleasedToggle = false;

    private boolean leftStickPressedToggle = false;
    private boolean leftStickReleasedToggle = false;
    private boolean rightStickPressedToggle = false;
    private boolean rightStickReleasedToggle = false;

    public Gamepad(com.qualcomm.robotcore.hardware.Gamepad gamepad) {
        gamepad.setLedColor(1.0 / 256.0, 0.0, 64.0 / 256.0, com.qualcomm.robotcore.hardware.Gamepad.LED_DURATION_CONTINUOUS);
        this.gamepad = gamepad;
    }

    private double applyDeadzone(double value) {
        return Math.abs(value) < DEADZONE ? 0 : value;
    }

    @Override
    public void periodic() {

        leftX = applyDeadzone(gamepad.left_stick_x);
        leftY = applyDeadzone(-gamepad.left_stick_y);
        rightX = applyDeadzone(gamepad.right_stick_x);
        rightY = applyDeadzone(-gamepad.right_stick_y);

        leftTrigger = applyDeadzone(gamepad.left_trigger);
        rightTrigger = applyDeadzone(gamepad.right_trigger);

        leftBumper = gamepad.left_bumper;
        rightBumper = gamepad.right_bumper;
        circle = gamepad.circle;
        cross = gamepad.cross;
        square = gamepad.square;
        triangle = gamepad.triangle;
        guide = gamepad.guide;

        dpadUp = gamepad.dpad_up;
        dpadDown = gamepad.dpad_down;
        dpadLeft = gamepad.dpad_left;
        dpadRight = gamepad.dpad_right;

        leftStickPressed = gamepad.left_stick_button;
        rightStickPressed = gamepad.right_stick_button;

        boolean currentLeftTriggerState = leftTrigger > 0;
        leftTriggerJustPressed = currentLeftTriggerState && !lastLeftTriggerState;
        leftTriggerJustReleased = !currentLeftTriggerState && lastLeftTriggerState;
        lastLeftTriggerState = currentLeftTriggerState;

        boolean currentRightTriggerState = rightTrigger > 0;
        rightTriggerJustPressed = currentRightTriggerState && !lastRightTriggerState;
        rightTriggerJustReleased = !currentRightTriggerState && lastRightTriggerState;
        lastRightTriggerState = currentRightTriggerState;

        leftBumperJustPressed = leftBumper && !lastLeftBumper;
        leftBumperJustReleased = !leftBumper && lastLeftBumper;
        lastLeftBumper = leftBumper;

        rightBumperJustPressed = rightBumper && !lastRightBumper;
        rightBumperJustReleased = !rightBumper && lastRightBumper;
        lastRightBumper = rightBumper;

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

        leftStickJustPressed = leftStickPressed && !lastLeftStickPressed;
        leftStickJustReleased = !leftStickPressed && lastLeftStickPressed;
        lastLeftStickPressed = leftStickPressed;

        rightStickJustPressed = rightStickPressed && !lastRightStickPressed;
        rightStickJustReleased = !rightStickPressed && lastRightStickPressed;
        lastRightStickPressed = rightStickPressed;

        if (leftTriggerJustPressed) leftTriggerPressedToggle = !leftTriggerPressedToggle;
        if (leftTriggerJustReleased) leftTriggerReleasedToggle = !leftTriggerReleasedToggle;
        if (rightTriggerJustPressed) rightTriggerPressedToggle = !rightTriggerPressedToggle;
        if (rightTriggerJustReleased) rightTriggerReleasedToggle = !rightTriggerReleasedToggle;

        if (leftBumperJustPressed) leftBumperPressedToggle = !leftBumperPressedToggle;
        if (leftBumperJustReleased) leftBumperReleasedToggle = !leftBumperReleasedToggle;
        if (rightBumperJustPressed) rightBumperPressedToggle = !rightBumperPressedToggle;
        if (rightBumperJustReleased) rightBumperReleasedToggle = !rightBumperReleasedToggle;

        if (circleJustPressed) circlePressedToggle = !circlePressedToggle;
        if (circleJustReleased) circleReleasedToggle = !circleReleasedToggle;
        if (crossJustPressed) crossPressedToggle = !crossPressedToggle;
        if (crossJustReleased) crossReleasedToggle = !crossReleasedToggle;
        if (squareJustPressed) squarePressedToggle = !squarePressedToggle;
        if (squareJustReleased) squareReleasedToggle = !squareReleasedToggle;
        if (triangleJustPressed) trianglePressedToggle = !trianglePressedToggle;
        if (triangleJustReleased) triangleReleasedToggle = !triangleReleasedToggle;
        if (guideJustPressed) guidePressedToggle = !guidePressedToggle;
        if (guideJustReleased) guideReleasedToggle = !guideReleasedToggle;

        if (dpadUpJustPressed) dpadUpPressedToggle = !dpadUpPressedToggle;
        if (dpadUpJustReleased) dpadUpReleasedToggle = !dpadUpReleasedToggle;
        if (dpadDownJustPressed) dpadDownPressedToggle = !dpadDownPressedToggle;
        if (dpadDownJustReleased) dpadDownReleasedToggle = !dpadDownReleasedToggle;
        if (dpadLeftJustPressed) dpadLeftPressedToggle = !dpadLeftPressedToggle;
        if (dpadLeftJustReleased) dpadLeftReleasedToggle = !dpadLeftReleasedToggle;
        if (dpadRightJustPressed) dpadRightPressedToggle = !dpadRightPressedToggle;
        if (dpadRightJustReleased) dpadRightReleasedToggle = !dpadRightReleasedToggle;

        if (leftStickJustPressed) leftStickPressedToggle = !leftStickPressedToggle;
        if (leftStickJustReleased) leftStickReleasedToggle = !leftStickReleasedToggle;
        if (rightStickJustPressed) rightStickPressedToggle = !rightStickPressedToggle;
        if (rightStickJustReleased) rightStickReleasedToggle = !rightStickReleasedToggle;
    }

    // ================================
    // =        STANDARD GETTERS       =
    // ================================

    public double getLeftX() {
        return leftX;
    }

    public double getLeftY() {
        return leftY;
    }

    public double getRightX() {
        return rightX;
    }

    public double getRightY() {
        return rightY;
    }

    public double getLeftTrigger() {
        return leftTrigger;
    }

    public double getRightTrigger() {
        return rightTrigger;
    }

    public boolean getLeftBumper() {
        return leftBumper;
    }

    public boolean getRightBumper() {
        return rightBumper;
    }

    public boolean getCircle() {
        return circle;
    }

    public boolean getCross() {
        return cross;
    }

    public boolean getSquare() {
        return square;
    }

    public boolean getTriangle() {
        return triangle;
    }

    public boolean getGuide() {
        return guide;
    }

    public boolean getDpadUp() {
        return dpadUp;
    }

    public boolean getDpadDown() {
        return dpadDown;
    }

    public boolean getDpadLeft() {
        return dpadLeft;
    }

    public boolean getDpadRight() {
        return dpadRight;
    }

    public boolean getLeftStickPressed() {
        return leftStickPressed;
    }

    public boolean getRightStickPressed() {
        return rightStickPressed;
    }

    // ================================
    // =     JUST PRESSED GETTERS      =
    // ================================

    public boolean getLeftTriggerJustPressed() {
        return leftTriggerJustPressed;
    }

    public boolean getRightTriggerJustPressed() {
        return rightTriggerJustPressed;
    }

    public boolean getLeftBumperJustPressed() {
        return leftBumperJustPressed;
    }

    public boolean getRightBumperJustPressed() {
        return rightBumperJustPressed;
    }

    public boolean getCircleJustPressed() {
        return circleJustPressed;
    }

    public boolean getCrossJustPressed() {
        return crossJustPressed;
    }

    public boolean getSquareJustPressed() {
        return squareJustPressed;
    }

    public boolean getTriangleJustPressed() {
        return triangleJustPressed;
    }

    public boolean getGuideJustPressed() {
        return guideJustPressed;
    }

    public boolean getDpadUpJustPressed() {
        return dpadUpJustPressed;
    }

    public boolean getDpadDownJustPressed() {
        return dpadDownJustPressed;
    }

    public boolean getDpadLeftJustPressed() {
        return dpadLeftJustPressed;
    }

    public boolean getDpadRightJustPressed() {
        return dpadRightJustPressed;
    }

    public boolean getLeftStickJustPressed() {
        return leftStickJustPressed;
    }

    public boolean getRightStickJustPressed() {
        return rightStickJustPressed;
    }

    // ================================
    // =     JUST RELEASED GETTERS     =
    // ================================

    public boolean getLeftTriggerJustReleased() {
        return leftTriggerJustReleased;
    }

    public boolean getRightTriggerJustReleased() {
        return rightTriggerJustReleased;
    }

    public boolean getLeftBumperJustReleased() {
        return leftBumperJustReleased;
    }

    public boolean getRightBumperJustReleased() {
        return rightBumperJustReleased;
    }

    public boolean getCircleJustReleased() {
        return circleJustReleased;
    }

    public boolean getCrossJustReleased() {
        return crossJustReleased;
    }

    public boolean getSquareJustReleased() {
        return squareJustReleased;
    }

    public boolean getTriangleJustReleased() {
        return triangleJustReleased;
    }

    public boolean getGuideJustReleased() {
        return guideJustReleased;
    }

    public boolean getDpadUpJustReleased() {
        return dpadUpJustReleased;
    }

    public boolean getDpadDownJustReleased() {
        return dpadDownJustReleased;
    }

    public boolean getDpadLeftJustReleased() {
        return dpadLeftJustReleased;
    }

    public boolean getDpadRightJustReleased() {
        return dpadRightJustReleased;
    }

    public boolean getLeftStickJustReleased() {
        return leftStickJustReleased;
    }

    public boolean getRightStickJustReleased() {
        return rightStickJustReleased;
    }

    // ================================
    // =        TOGGLE GETTERS/SETTERS =
    // ================================

    public boolean getLeftTriggerPressedToggle() {
        return leftTriggerPressedToggle;
    }

    public void setLeftTriggerPressedToggle(boolean v) {
        leftTriggerPressedToggle = v;
    }

    public boolean getLeftTriggerReleasedToggle() {
        return leftTriggerReleasedToggle;
    }

    public void setLeftTriggerReleasedToggle(boolean v) {
        leftTriggerReleasedToggle = v;
    }

    public boolean getRightTriggerPressedToggle() {
        return rightTriggerPressedToggle;
    }

    public void setRightTriggerPressedToggle(boolean v) {
        rightTriggerPressedToggle = v;
    }

    public boolean getRightTriggerReleasedToggle() {
        return rightTriggerReleasedToggle;
    }

    public void setRightTriggerReleasedToggle(boolean v) {
        rightTriggerReleasedToggle = v;
    }

    public boolean getLeftBumperPressedToggle() {
        return leftBumperPressedToggle;
    }

    public void setLeftBumperPressedToggle(boolean v) {
        leftBumperPressedToggle = v;
    }

    public boolean getLeftBumperReleasedToggle() {
        return leftBumperReleasedToggle;
    }

    public void setLeftBumperReleasedToggle(boolean v) {
        leftBumperReleasedToggle = v;
    }

    public boolean getRightBumperPressedToggle() {
        return rightBumperPressedToggle;
    }

    public void setRightBumperPressedToggle(boolean v) {
        rightBumperPressedToggle = v;
    }

    public boolean getRightBumperReleasedToggle() {
        return rightBumperReleasedToggle;
    }

    public void setRightBumperReleasedToggle(boolean v) {
        rightBumperReleasedToggle = v;
    }

    public boolean getCirclePressedToggle() {
        return circlePressedToggle;
    }

    public void setCirclePressedToggle(boolean v) {
        circlePressedToggle = v;
    }

    public boolean getCircleReleasedToggle() {
        return circleReleasedToggle;
    }

    public void setCircleReleasedToggle(boolean v) {
        circleReleasedToggle = v;
    }

    public boolean getCrossPressedToggle() {
        return crossPressedToggle;
    }

    public void setCrossPressedToggle(boolean v) {
        crossPressedToggle = v;
    }

    public boolean getCrossReleasedToggle() {
        return crossReleasedToggle;
    }

    public void setCrossReleasedToggle(boolean v) {
        crossReleasedToggle = v;
    }

    public boolean getSquarePressedToggle() {
        return squarePressedToggle;
    }

    public void setSquarePressedToggle(boolean v) {
        squarePressedToggle = v;
    }

    public boolean getSquareReleasedToggle() {
        return squareReleasedToggle;
    }

    public void setSquareReleasedToggle(boolean v) {
        squareReleasedToggle = v;
    }

    public boolean getTrianglePressedToggle() {
        return trianglePressedToggle;
    }

    public void setTrianglePressedToggle(boolean v) {
        trianglePressedToggle = v;
    }

    public boolean getTriangleReleasedToggle() {
        return triangleReleasedToggle;
    }

    public void setTriangleReleasedToggle(boolean v) {
        triangleReleasedToggle = v;
    }

    public boolean getGuidePressedToggle() {
        return guidePressedToggle;
    }

    public void setGuidePressedToggle(boolean v) {
        guidePressedToggle = v;
    }

    public boolean getGuideReleasedToggle() {
        return guideReleasedToggle;
    }

    public void setGuideReleasedToggle(boolean v) {
        guideReleasedToggle = v;
    }

    public boolean getDpadUpPressedToggle() {
        return dpadUpPressedToggle;
    }

    public void setDpadUpPressedToggle(boolean v) {
        dpadUpPressedToggle = v;
    }

    public boolean getDpadUpReleasedToggle() {
        return dpadUpReleasedToggle;
    }

    public void setDpadUpReleasedToggle(boolean v) {
        dpadUpReleasedToggle = v;
    }

    public boolean getDpadDownPressedToggle() {
        return dpadDownPressedToggle;
    }

    public void setDpadDownPressedToggle(boolean v) {
        dpadDownPressedToggle = v;
    }

    public boolean getDpadDownReleasedToggle() {
        return dpadDownReleasedToggle;
    }

    public void setDpadDownReleasedToggle(boolean v) {
        dpadDownReleasedToggle = v;
    }

    public boolean getDpadLeftPressedToggle() {
        return dpadLeftPressedToggle;
    }

    public void setDpadLeftPressedToggle(boolean v) {
        dpadLeftPressedToggle = v;
    }

    public boolean getDpadLeftReleasedToggle() {
        return dpadLeftReleasedToggle;
    }

    public void setDpadLeftReleasedToggle(boolean v) {
        dpadLeftReleasedToggle = v;
    }

    public boolean getDpadRightPressedToggle() {
        return dpadRightPressedToggle;
    }

    public void setDpadRightPressedToggle(boolean v) {
        dpadRightPressedToggle = v;
    }

    public boolean getDpadRightReleasedToggle() {
        return dpadRightReleasedToggle;
    }

    public void setDpadRightReleasedToggle(boolean v) {
        dpadRightReleasedToggle = v;
    }

    public boolean getLeftStickPressedToggle() {
        return leftStickPressedToggle;
    }

    public void setLeftStickPressedToggle(boolean v) {
        leftStickPressedToggle = v;
    }

    public boolean getLeftStickReleasedToggle() {
        return leftStickReleasedToggle;
    }

    public void setLeftStickReleasedToggle(boolean v) {
        leftStickReleasedToggle = v;
    }

    public boolean getRightStickPressedToggle() {
        return rightStickPressedToggle;
    }

    public void setRightStickPressedToggle(boolean v) {
        rightStickPressedToggle = v;
    }

    public boolean getRightStickReleasedToggle() {
        return rightStickReleasedToggle;
    }

    public void setRightStickReleasedToggle(boolean v) {
        rightStickReleasedToggle = v;
    }

    public void resetAllToggles() {

        leftTriggerPressedToggle = false;
        leftTriggerReleasedToggle = false;
        rightTriggerPressedToggle = false;
        rightTriggerReleasedToggle = false;

        leftBumperPressedToggle = false;
        leftBumperReleasedToggle = false;
        rightBumperPressedToggle = false;
        rightBumperReleasedToggle = false;

        circlePressedToggle = false;
        circleReleasedToggle = false;
        crossPressedToggle = false;
        crossReleasedToggle = false;
        squarePressedToggle = false;
        squareReleasedToggle = false;
        trianglePressedToggle = false;
        triangleReleasedToggle = false;
        guidePressedToggle = false;
        guideReleasedToggle = false;

        dpadUpPressedToggle = false;
        dpadUpReleasedToggle = false;
        dpadDownPressedToggle = false;
        dpadDownReleasedToggle = false;
        dpadLeftPressedToggle = false;
        dpadLeftReleasedToggle = false;
        dpadRightPressedToggle = false;
        dpadRightReleasedToggle = false;

        leftStickPressedToggle = false;
        leftStickReleasedToggle = false;
        rightStickPressedToggle = false;
        rightStickReleasedToggle = false;
    }


    public void resetAllPressedToggles() {

        leftTriggerPressedToggle = false;
        rightTriggerPressedToggle = false;

        leftBumperPressedToggle = false;
        rightBumperPressedToggle = false;

        circlePressedToggle = false;
        crossPressedToggle = false;
        squarePressedToggle = false;
        trianglePressedToggle = false;
        guidePressedToggle = false;

        dpadUpPressedToggle = false;
        dpadDownPressedToggle = false;
        dpadLeftPressedToggle = false;
        dpadRightPressedToggle = false;

        leftStickPressedToggle = false;
        rightStickPressedToggle = false;
    }

    public void resetAllReleasedToggles() {

        leftTriggerReleasedToggle = false;
        rightTriggerReleasedToggle = false;

        leftBumperReleasedToggle = false;
        rightBumperReleasedToggle = false;

        circleReleasedToggle = false;
        crossReleasedToggle = false;
        squareReleasedToggle = false;
        triangleReleasedToggle = false;
        guideReleasedToggle = false;

        dpadUpReleasedToggle = false;
        dpadDownReleasedToggle = false;
        dpadLeftReleasedToggle = false;
        dpadRightReleasedToggle = false;

        leftStickReleasedToggle = false;
        rightStickReleasedToggle = false;
    }


    // ================================
    // =            RUMBLE             =
    // ================================

    public void rumble(int milliseconds) {
        gamepad.rumble(milliseconds);
    }

    public void rumble(double left, double right, int milliseconds) {
        gamepad.rumble(left, right, milliseconds);
    }

    public void rumbleBlips(int count) {
        gamepad.rumbleBlips(count);
    }

    public void stopRumble() {
        gamepad.stopRumble();
    }
}

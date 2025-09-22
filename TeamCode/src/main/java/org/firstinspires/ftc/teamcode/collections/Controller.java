package org.firstinspires.ftc.teamcode.collections;

import com.qualcomm.robotcore.hardware.Gamepad;

// TODO: Add multiple gamepad support, only gamepad one works rn
public class Controller {
    Gamepad gamepad1;
    Gamepad gamepad2;

    private boolean lastSquare = false;
    private boolean lastCircle = false;
    private boolean lastCross = false;
    private boolean lastTriangle = false;
    public boolean square = false;
    public boolean circle = false;
    public boolean cross = false;
    public boolean triangle = false;

    public void update() {
        lastSquare = square;
        lastCircle = circle;
        lastCross = cross;
        lastTriangle = triangle;
        square = gamepad1.square;
        circle = gamepad1.circle;
        cross = gamepad1.cross;
        triangle = gamepad1.triangle;
    }

    public boolean isSquareJustPressed() {
        return square && !lastSquare;
    }

    public boolean isCircleJustPressed() {
        return circle && !lastCircle;
    }

    public boolean isCrossJustPressed() {
        return cross && !lastCross;
    }

    public boolean isTriangleJustPressed() {
        return triangle && !lastTriangle;
    }

    public void init(Gamepad gamepad1, Gamepad gamepad2) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }
}

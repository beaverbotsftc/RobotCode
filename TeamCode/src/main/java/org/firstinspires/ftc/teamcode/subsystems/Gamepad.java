package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.qualcomm.robotcore.util.RobotLog;

import org.beaverbots.BeaverCommand.Subsystem;

public class Gamepad implements Subsystem {
    private com.qualcomm.robotcore.hardware.Gamepad gamepad;

    private double leftX = 0;
    private double leftY = 0;
    private double rightX = 0;
    private double rightY = 0;

    private double leftTrigger = 0;
    private double rightTrigger = 0;

    private boolean leftBumper = false;
    private boolean rightBumper = false;

    private boolean circle = false;
    private boolean cross = false;
    private boolean square = false;
    private boolean triangle = false;

    public Gamepad(com.qualcomm.robotcore.hardware.Gamepad gamepad) {
        gamepad.setLedColor(1 / 256.0, 0.0, 64 / 256.0, com.qualcomm.robotcore.hardware.Gamepad.LED_DURATION_CONTINUOUS);
        this.gamepad = gamepad;
    }

    public void periodic() {
        leftX = gamepad.left_stick_x;
        leftY = -gamepad.left_stick_y;
        rightX = gamepad.right_stick_x;
        rightY = -gamepad.right_stick_y;

        leftTrigger = gamepad.left_trigger;
        rightTrigger = gamepad.right_trigger;

        leftBumper = gamepad.left_bumper;
        rightBumper = gamepad.right_bumper;

        circle = gamepad.circle;
        cross = gamepad.cross;
        square = gamepad.square;
        triangle = gamepad.triangle;
    }

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

    public boolean getCircle() {return circle;}
    public boolean getCross() {return cross;}
    public boolean getSquare() {return square;}
    public boolean getTriangle() {return triangle;}
}

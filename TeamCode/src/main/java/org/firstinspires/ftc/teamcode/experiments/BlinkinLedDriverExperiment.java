package org.firstinspires.ftc.teamcode.experiments;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@TeleOp
public class BlinkinLedDriverExperiment extends OpMode {

    /*
     * Rate limit gamepad button presses to every 500ms.
     */
    private static final int GAMEPAD_LOCKOUT = 500;

    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;

    Telemetry.Item patternName;
    Deadline gamepadRateLimit;

    @Override
    public void init() {

        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
        blinkinLedDriver.setPattern(pattern);

        patternName = telemetry.addData("Pattern", pattern.toString());

        gamepadRateLimit = new Deadline(GAMEPAD_LOCKOUT, TimeUnit.MILLISECONDS);
    }

    @Override
    public void loop() {
        handleGamepad();
    }

    private void handleGamepad() {

        if (!gamepadRateLimit.hasExpired()) {
            return;
        }

        if (gamepad1.left_bumper) {
            pattern = pattern.previous();
            displayPattern();
            gamepadRateLimit.reset();
        }
        else if (gamepad1.right_bumper) {
            pattern = pattern.next();
            displayPattern();
            gamepadRateLimit.reset();
        }
    }

    private void displayPattern() {
        blinkinLedDriver.setPattern(pattern);
        patternName.setValue(pattern.toString());
    }
}

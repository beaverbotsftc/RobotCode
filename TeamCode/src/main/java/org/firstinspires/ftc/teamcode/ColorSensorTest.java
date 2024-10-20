package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

@TeleOp(name="aColorSensorTest", group="Linear OpMode")
public class ColorSensorTest extends LinearOpMode{
    // Define a variable for our color sensor
    ColorSensor color;
    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;
    private double distance = 0;

    @Override
    public void runOpMode() {
        // Get the color sensor from hardwareMap
        color = hardwareMap.get(ColorSensor.class, "Color");
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        pattern = RevBlinkinLedDriver.BlinkinPattern.CP1_2_SPARKLE_1_ON_2;
        blinkinLedDriver.setPattern(pattern);

        color.enableLed(true);
        // Wait for the Play button to be pressed
        waitForStart();


        // While the Op Mode is running, update the telemetry values.
        while (opModeIsActive()) {

            distance = ((DistanceSensor) color).getDistance(DistanceUnit.CM);

            if(distance <= 3.5) {
                if ((color.blue() > color.red() * 0.9 + color.green() * 0.9)) {
                    telemetry.addLine("I see more blue than anything else");
                    pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
                } else if (color.red() > color.blue() * 0.9 + color.green() * 0.9) {
                    telemetry.addLine("I see more red than the rest");
                    pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
                } else if ((color.green() > color.red() * 1.2 + color.blue() * 1.2)) {
                    telemetry.addLine("I see more green than anything else");
                    pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
                } else if ((color.red() - color.green()) <= color.red() / 10 && (color.red() - color.green()) <= color.green() / 10 && (color.blue() * 5) <= (color.green() + color.red())) {
                    telemetry.addLine("I see yellow I think ");
                    pattern = RevBlinkinLedDriver.BlinkinPattern.GOLD;
                } else {
                    telemetry.addLine("I don't see anything");
                    pattern = RevBlinkinLedDriver.BlinkinPattern.VIOLET;
                }
            } else {
                telemetry.addLine("The things are too far away!");
                pattern = RevBlinkinLedDriver.BlinkinPattern.HOT_PINK;
            }
            blinkinLedDriver.setPattern(pattern);

            if(gamepad1.dpad_up){
                gamepad1.setLedColor(0,10000,0,2000);
                telemetry.addData("DPAD UP", gamepad1.dpad_up);
            } else if (gamepad1.dpad_down) {
                telemetry.addData("DPAD DOWN", gamepad1.dpad_down);
                gamepad1.setLedColor(255,0,0,1000);
            }
/*
            if(gamepad1.triangle){
                gamepad1.rumble(1,0, 1000);
            } else if (gamepad1.circle) {
                gamepad1.rumble(0,1,1000);
            } else if(gamepad1.square){
                gamepad1.rumble(1000);
            }e  lse if(gamepad1.cross){
                gamepad1.rumbleBlips(5);
            }
*/

            telemetry.addData("Red", color.red());
            telemetry.addData("Green", color.green());
            telemetry.addData("Blue", color.blue());
            telemetry.addData("Distance (cm)", distance);
            telemetry.update();
        }
    }
}

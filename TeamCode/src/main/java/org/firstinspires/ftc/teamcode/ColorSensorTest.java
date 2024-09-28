package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
public class ColorSensorTest extends LinearOpMode{
    // Define a variable for our color sensor
    ColorSensor color;

    @Override
    public void runOpMode() {
        // Get the color sensor from hardwareMap
        color = hardwareMap.get(ColorSensor.class, "Color");

        // Wait for the Play button to be pressed
        waitForStart();

        // While the Op Mode is running, update the telemetry values.
        while (opModeIsActive()) {

            double colorSensorMultiplier = 0.9;
            if (color.red() * colorSensorMultiplier > color.blue() && color.red() * colorSensorMultiplier > color.green()){
                telemetry.addLine("I see more red than the rest");
                gamepad1.setLedColor(1,0,0,250);
            }else if ((color.blue() * colorSensorMultiplier > color.red() && color.blue() * colorSensorMultiplier > color.green())) {
                telemetry.addLine("I see more blue than anything else");
                gamepad1.setLedColor(0,0,1,250);
            } else if ((color.green() * colorSensorMultiplier > color.red() && color.green() * colorSensorMultiplier > color.blue())) {
                telemetry.addLine("I see more green than anything else");
                gamepad1.setLedColor(0,1,0,250);
            }else{
                telemetry.addLine("I don't see anything");
            }

            telemetry.addData("Red", color.red());
            telemetry.addData("Green", color.green());
            telemetry.addData("Blue", color.blue());
            telemetry.update();
        }
    }
}




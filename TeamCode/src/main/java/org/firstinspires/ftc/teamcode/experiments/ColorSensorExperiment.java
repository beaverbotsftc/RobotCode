package org.firstinspires.ftc.teamcode.experiments;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Color Sensor Experiment", group = "Sensor")
public class ColorSensorExperiment extends LinearOpMode {

  NormalizedColorSensor colorSensor;


  @Override public void runOpMode() {

    float gain = 59;
    final float[] hsvValues = new float[3];
    colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color2");

    waitForStart();

    // Loop until we are asked to stop
    while (opModeIsActive()) {

      telemetry.addData("Gain", gain);
      colorSensor.setGain(gain);

      NormalizedRGBA colors = colorSensor.getNormalizedColors();

      Color.colorToHSV(colors.toColor(), hsvValues);

      telemetry.addLine()
              .addData("Hue", "%.3f", hsvValues[0])
              .addData("Saturation", "%.3f", hsvValues[1])
              .addData("Value", "%.3f", hsvValues[2]);

      double dist = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);
      telemetry.addData("Distance (cm)", "%.3f", dist);


      telemetry.addLine("");
/*
      if(Math.abs(hsvValues[0] - 230)/100 <= 0.06){
        if(hsvValues[2] > 0.30 && dist <= 6.5){
          telemetry.addLine("Currently detecting PURPLE");
        }else{
          telemetry.addLine("almost PURPLE");
        }
      }else if(Math.abs(hsvValues[0] - 160)/100 <= 0.06){
        if(hsvValues[2] > 0.65 && dist <= 6.5){
          telemetry.addLine("Currently detecting GREEN");
        }else{
          telemetry.addLine("almost GREEN");
        }
      }else{
        telemetry.addLine("I'm not seeing Green or PURPLE");
      }
 */
      if(hsvValues[2] < 0.1 && dist >= 7.5){
        telemetry.addLine("There ain't no ball");
      } else {
        if(Math.abs(hsvValues[0] - 160)/100 <= 0.06){
          telemetry.addLine("GREEN");

        } else if (Math.abs(hsvValues[0] - 230)/100 <= 0.06) {
          telemetry.addLine("PURPLE");

        }else{
          telemetry.addLine("Somehow Nothing?");

        }
      }

      telemetry.update();
    }
  }
}

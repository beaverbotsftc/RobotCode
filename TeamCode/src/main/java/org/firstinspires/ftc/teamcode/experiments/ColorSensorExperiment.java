package org.firstinspires.ftc.teamcode.experiments;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Color Sensor Experiment", group = "Sensor")
public class ColorSensorExperiment extends LinearOpMode {

  NormalizedColorSensor colorSensor;
  NormalizedColorSensor colorSensor2;
  boolean hasBall1 = false;
  boolean hasBall2 = false;

  @Override
  public void runOpMode() {

    float gain = 59;

    final float[] hsvValues1 = new float[3];
    final float[] hsvValues2 = new float[3];

    colorSensor  = hardwareMap.get(NormalizedColorSensor.class, "color");
    colorSensor2 = hardwareMap.get(NormalizedColorSensor.class, "color2");

    colorSensor.setGain(gain);
    colorSensor2.setGain(gain);

    waitForStart();

    while (opModeIsActive()) {

      telemetry.addData("Gain", gain);

      /* ================= SENSOR 1 ================= */
      NormalizedRGBA colors1 = colorSensor.getNormalizedColors();
      Color.colorToHSV(colors1.toColor(), hsvValues1);
      double dist1 = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);

      telemetry.addLine("=== Color Sensor 1 ===");
      telemetry.addData("Hue", "%.3f", hsvValues1[0]);
      telemetry.addData("Sat", "%.3f", hsvValues1[1]);
      telemetry.addData("Val", "%.3f", hsvValues1[2]);
      telemetry.addData("Distance (cm)", "%.3f", dist1);

      if (dist1 >= 8) {
        telemetry.addLine("Ball: NONE");
        hasBall1 = false;
      } else {
        telemetry.addLine("Ball: PRESENT");
        hasBall1 = true;
      }

      telemetry.addLine("");

      /* ================= SENSOR 2 ================= */
      NormalizedRGBA colors2 = colorSensor2.getNormalizedColors();
      Color.colorToHSV(colors2.toColor(), hsvValues2);
      double dist2 = ((DistanceSensor) colorSensor2).getDistance(DistanceUnit.CM);

      telemetry.addLine("=== Color Sensor 2 ===");
      telemetry.addData("Hue", "%.3f", hsvValues2[0]);
      telemetry.addData("Sat", "%.3f", hsvValues2[1]);
      telemetry.addData("Val", "%.3f", hsvValues2[2]);
      telemetry.addData("Distance (cm)", "%.3f", dist2);

      if (dist2 >= 5) {
        telemetry.addLine("Ball: NONE");
        hasBall2 = false;
      } else {
        telemetry.addLine("Ball: PRESENT");
        hasBall2 = true;
      }

      if(hasBall1 && hasBall2){
        telemetry.addLine("Has three balls");
      }

      telemetry.update();
    }
  }
}

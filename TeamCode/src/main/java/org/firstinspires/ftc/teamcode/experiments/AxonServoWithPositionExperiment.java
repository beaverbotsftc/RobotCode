package org.firstinspires.ftc.teamcode.experiments;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.beaverbots.beaver.command.CommandRuntimeOpMode;
import org.beaverbots.beaver.command.HardwareManager;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class AxonServoWithPositionExperiment extends CommandRuntimeOpMode {
  AnalogInput encoder;

  public void onInit() {
    encoder = HardwareManager.claim(AnalogInput.class, "servo encoder");
  }
  public void onStart() {}
}

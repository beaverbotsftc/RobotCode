package org.firstinspires.ftc.teamcode.experiments;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(group = "Sensor")
public class SimpleAxonWireReadExperiment extends LinearOpMode {

  AnalogInput encoder0;
  CRServo servo0;

  double previousAngle = 0.0;
  double totalEncoderRotation = 0.0;

  // Gear ratio: encoder on 24T, output on 38T
  static final double GEAR_RATIO = 24.0 / 38.0;

  @Override
  public void runOpMode() {

    encoder0 = hardwareMap.get(AnalogInput.class, "encoder0");
    servo0 = hardwareMap.get(CRServo.class, "servo0");

    waitForStart();

    // Initialize reference angle
    previousAngle = getAngle();

    while (opModeIsActive()) {

      // --- Servo control ---
      if (gamepad1.triangle) {
        servo0.setPower(1);
      } else if (gamepad1.cross) {
        servo0.setPower(-1);
      } else {
        servo0.setPower(0);
      }

      // --- Encoder reading ---
      double currentAngle = getAngle();
      double delta = currentAngle - previousAngle;

      // --- Wraparound handling ---
      if (delta > 180) {
        delta -= 360;
      } else if (delta < -180) {
        delta += 360;
      }

      totalEncoderRotation += delta;
      previousAngle = currentAngle;

      // Convert to output rotation
      double outputRotation = totalEncoderRotation * GEAR_RATIO;

      // --- Telemetry ---
      telemetry.addLine("ENCODER 0:");
      telemetry.addData("Voltage", encoder0.getVoltage());
      telemetry.addData("Angle (deg)", currentAngle);
      telemetry.addData("Encoder Rotation (deg)", totalEncoderRotation);
      telemetry.addData("Output Rotation (deg)", outputRotation);

      telemetry.addLine("");
      telemetry.addData("Servo Power", servo0.getPower());

      telemetry.update();
    }
  }

  private double getAngle() {
    return (encoder0.getVoltage() / encoder0.getMaxVoltage()) * 360.0;
  }
}

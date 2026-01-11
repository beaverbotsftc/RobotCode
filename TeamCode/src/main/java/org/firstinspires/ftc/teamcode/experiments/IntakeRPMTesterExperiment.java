package org.firstinspires.ftc.teamcode.experiments;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Autonomous(name="IntakeRPMTesterExperiment", group="Examples")
public class IntakeRPMTesterExperiment extends LinearOpMode {

    private DcMotorEx motor;
    private DcMotorEx stopper;
    private double power = 1;
    private static final double TICKS_PER_REV = 145.1;

    @Override
    public void runOpMode(){
        motor = hardwareMap.get(DcMotorEx.class, "intake");
        motor.setDirection(DcMotor.Direction.FORWARD);
        stopper = hardwareMap.get(DcMotorEx.class, "stopper");

        // Reset encoders
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        stopper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        stopper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            double speed = -gamepad1.left_stick_y;

            // Increment power when triangle is pressed
            if (gamepad1.rightBumperWasPressed()) {
                if (power < 1) {
                    power += 0.05;
                }
            }

            // Decrement power when X is pressed
            if (gamepad1.leftBumperWasPressed()) {
                if(power > 0){
                    power -= 0.05;
                }
            }


            // Apply motor power
            double outputPower = power * speed;
            motor.setPower(outputPower);

            stopper.setPower(-gamepad1.right_stick_y * 0.8);

            // Calculate RPMs
            double rpm1 = ticksPerSecondToRPM(motor.getVelocity());
            double encoder = motor.getCurrentPosition();

            // Telemetry
            telemetry.addData("Joystick Value", "%.2f", gamepad1.left_stick_y);
            telemetry.addData("Motor Power Limit", "%.2f", power);
            telemetry.addData("Motor 1 RPM", "%.1f", rpm1);
            telemetry.addData("Motor 1 encode", "%.1f", encoder);
            telemetry.addData("Motor 1 Current(Amps)", "%.1f", motor.getCurrent(CurrentUnit.AMPS));

            telemetry.update();
        }
    }

    private double ticksPerSecondToRPM(double ticksPerSecond) {
        return (ticksPerSecond / TICKS_PER_REV) * 60.0;
    }
}

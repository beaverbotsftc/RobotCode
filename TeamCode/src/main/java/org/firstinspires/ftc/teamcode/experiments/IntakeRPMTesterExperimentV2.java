package org.firstinspires.ftc.teamcode.experiments;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name="IntakeRPMTesterExperimentV2", group="Examples")
public class IntakeRPMTesterExperimentV2 extends LinearOpMode {

    private DcMotorEx motor;
    private DcMotorEx stopper;
    private double power = 1;
    @Override
    public void runOpMode(){
        motor = hardwareMap.get(DcMotorEx.class, "intake");
        motor.setDirection(DcMotor.Direction.REVERSE);
        stopper = hardwareMap.get(DcMotorEx.class, "stopper");

        // Reset encoders
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        stopper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
            stopper.setPower(outputPower);



            // Telemetry
            telemetry.addData("Joystick Value", "%.2f", gamepad1.left_stick_y);
            telemetry.addData("Motor Power Limit", "%.2f", power);
            telemetry.addData("Motor 1 Current(Amps)", "%.1f", motor.getCurrent(CurrentUnit.AMPS));

            telemetry.update();
        }
    }
}

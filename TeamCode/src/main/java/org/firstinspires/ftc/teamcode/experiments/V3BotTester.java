package org.firstinspires.ftc.teamcode.experiments;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name="V3BotTester", group="Examples")
public class V3BotTester extends LinearOpMode {

    private DcMotorEx transfer;
    private DcMotorEx transfer2;
    private DcMotorEx shooter;
    private DcMotorEx shooter2;
    private Servo stopper;
    private double shooterPower = 0.5;
    @Override
    public void runOpMode(){
        transfer = hardwareMap.get(DcMotorEx.class, "transfer1");
        transfer.setDirection(DcMotor.Direction.REVERSE);
        transfer2 = hardwareMap.get(DcMotorEx.class, "transfer2");

        shooter = hardwareMap.get(DcMotorEx.class, "left shooter");
        shooter2 = hardwareMap.get(DcMotorEx.class, "right shooter");
        shooter2.setDirection(DcMotor.Direction.REVERSE);

        stopper = hardwareMap.get(Servo.class, "stopper");

        // Set encoders
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            // Apply motor power
            if(gamepad1.right_trigger > 0.05 || gamepad1.left_trigger > 0.05){
                transfer.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
                transfer2.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
            }else{
                transfer.setPower(0);
                transfer2.setPower(0);
            }

            if(gamepad1.yWasPressed()){
                shooterPower += 0.05;
            }else if(gamepad1.aWasPressed()){
                shooterPower -= 0.05;
            }

            if(gamepad1.left_bumper){
                shooter.setPower(shooterPower);
                shooter2.setPower(shooterPower);
            }else{
                shooter.setPower(0);
                shooter2.setPower(0);
            }


            //         turretPos = ((0.65 * (-gamepad1.right_stick_y) + 1) / 2.0);
            if(gamepad1.dpadUpWasPressed()){
                stopper.setPosition(0.4);
            }else if(gamepad1.dpadDownWasPressed()){
                stopper.setPosition(0.8);
            }


            // Telemetry
            telemetry.addData("Intake power", "%.2f", transfer.getPower());
            telemetry.addData("Stopper power", "%.2f", transfer2.getPower());
            telemetry.addData("Shooter power", "%.2f", shooterPower);
            telemetry.addData("Turret Position", "%.2f", stopper.getPosition());

            telemetry.addData("Shooter TPS", shooter.getVelocity());
            telemetry.addData("Transfer TPS", transfer.getVelocity());
            telemetry.addData("Transfer Current Draw", transfer.getCurrent(CurrentUnit.AMPS));

            telemetry.update();
        }
    }
}

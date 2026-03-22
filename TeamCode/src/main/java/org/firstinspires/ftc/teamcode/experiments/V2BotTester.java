package org.firstinspires.ftc.teamcode.experiments;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name="V2BotTester", group="Examples")
public class V2BotTester extends LinearOpMode {

    private DcMotorEx intake;
    private DcMotorEx stopper;
    private DcMotorEx shooter;
    private DcMotorEx shooter2;
    private Servo turret;
    private Servo turret2;
    private double shooterPower = 0.5;
    private double turretPos = 0.5;
    @Override
    public void runOpMode(){
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotor.Direction.FORWARD);
        stopper = hardwareMap.get(DcMotorEx.class, "stopper");

        shooter = hardwareMap.get(DcMotorEx.class, "left shooter");
        shooter.setDirection(DcMotor.Direction.REVERSE);
        shooter2 = hardwareMap.get(DcMotorEx.class, "right shooter");

        turret = hardwareMap.get(Servo.class, "left turret");
     //   turret.setDirection(Servo.Direction.REVERSE);
        turret2 = hardwareMap.get(Servo.class, "right turret");

        // Set encoders
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        stopper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            // Apply motor power
            if(gamepad1.right_bumper){
                intake.setPower(1);
                stopper.setPower(1);
            }else if(gamepad1.right_trigger > 0.05 || gamepad1.left_trigger > 0.05){
                intake.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
                stopper.setPower(-0.8);
            }else{
                intake.setPower(0);
                stopper.setPower(0);
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
                turretPos += 0.03;
            }else if(gamepad1.dpadDownWasPressed()){
                turretPos -= 0.03;
            }
            turret.setPosition(turretPos);
            turret2.setPosition(turretPos);


            // Telemetry
            telemetry.addData("Intake power", "%.2f", intake.getPower());
            telemetry.addData("Stopper power", "%.2f", stopper.getPower());
            telemetry.addData("Shooter power", "%.2f", shooterPower);
            telemetry.addData("Turret Position", "%.2f", turretPos);

            telemetry.update();
        }
    }
}

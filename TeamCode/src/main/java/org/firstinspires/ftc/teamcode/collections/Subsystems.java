package org.firstinspires.ftc.teamcode.collections;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Subsystems {
    public DcMotor leftVerSlide = null;
    public DcMotor rightVerSlide = null;
    public DcMotor rightHorSlide = null;
    public DcMotor leftHorSlide = null;
    public Servo intakeRotationServo = null;
    public Servo intakeClawServo = null;
    public Servo intakeArmServo = null;

    public void init(HardwareMap hardwareMap) {
        leftVerSlide = hardwareMap.get(DcMotor.class, "left ver slide");
        rightVerSlide = hardwareMap.get(DcMotor.class, "right ver slide");
        rightHorSlide = hardwareMap.get(DcMotor.class, "right hor slide");
        leftHorSlide = hardwareMap.get(DcMotor.class, "left hor slide");

        leftVerSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        rightVerSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        rightHorSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        leftHorSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        leftVerSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftVerSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightVerSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightVerSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightHorSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightHorSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftHorSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftHorSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeRotationServo = hardwareMap.get(Servo.class, "intake rotation servo");
        intakeRotationServo.setDirection(Servo.Direction.FORWARD);

        intakeClawServo = hardwareMap.get(Servo.class, "intake claw servo");
        intakeClawServo.setDirection(Servo.Direction.FORWARD);

        intakeArmServo = hardwareMap.get(Servo.class, "intake arm servo");
        intakeArmServo.setDirection(Servo.Direction.FORWARD);

    }
    public void verticalSlide(double power){
        leftVerSlide.setPower(power);
        rightVerSlide.setPower(power);
    }

    public void horizontalSlide(double power){
        leftHorSlide.setPower(power);
        rightHorSlide.setPower(power);
    }
}

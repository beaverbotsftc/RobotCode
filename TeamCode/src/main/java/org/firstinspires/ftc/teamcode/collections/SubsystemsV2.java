package org.firstinspires.ftc.teamcode.collections;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashMap;

public class SubsystemsV2 {
    public DcMotor leftVerSlide = null;
    public DcMotor rightVerSlide = null;
    public DcMotor leftHorSlide = null;
    public DcMotor rightHorSlide = null;
    public Servo intakeRotationServo = null;
    public Servo intakeClawServo = null;
    public Servo intakeArmServo = null;
    public Servo outtakeRotationServo = null;
    public Servo outtakeClawServo = null;
    public Servo outtakeWristServo = null;
    public Servo outtakeLeftArmServo = null;
    public Servo outtakeRightArmServo = null;


    public void init(HardwareMap hardwareMap) {
        leftVerSlide = hardwareMap.get(DcMotor.class, "left ver slide");
        rightVerSlide = hardwareMap.get(DcMotor.class, "right ver slide");
        leftHorSlide = hardwareMap.get(DcMotor.class, "left hor slide");
        rightHorSlide = hardwareMap.get(DcMotor.class, "right hor slide");

        leftVerSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        rightVerSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        leftHorSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        rightHorSlide.setDirection(DcMotorSimple.Direction.FORWARD);

        leftVerSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftVerSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightVerSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightVerSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftHorSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftHorSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightHorSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightHorSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeRotationServo = hardwareMap.get(Servo.class, "intake rotation servo");
        intakeRotationServo.setDirection(Servo.Direction.FORWARD);

        intakeClawServo = hardwareMap.get(Servo.class, "intake claw servo");
        intakeClawServo.setDirection(Servo.Direction.FORWARD);

        intakeArmServo = hardwareMap.get(Servo.class, "intake arm servo");
        intakeArmServo.setDirection(Servo.Direction.FORWARD);

        // Initialize outtakeRotationServo
        outtakeRotationServo = hardwareMap.get(Servo.class, "outtake rotation servo");
        outtakeRotationServo.setDirection(Servo.Direction.FORWARD);

        // Initialize outtakeClawServo
        outtakeClawServo = hardwareMap.get(Servo.class, "outtake claw servo");
        outtakeClawServo.setDirection(Servo.Direction.FORWARD);

        // Initialize outtakeWristServo
        outtakeWristServo = hardwareMap.get(Servo.class, "outtake wrist servo");
        outtakeWristServo.setDirection(Servo.Direction.FORWARD);

        // Initialize outtakeLeftArmServo
        outtakeLeftArmServo = hardwareMap.get(Servo.class, "outtake left arm servo");
        outtakeLeftArmServo.setDirection(Servo.Direction.FORWARD);

        // Initialize outtakeRightArmServo
        outtakeRightArmServo = hardwareMap.get(Servo.class, "outtake right arm servo");
        outtakeRightArmServo.setDirection(Servo.Direction.REVERSE);
    }
    public void verticalSlide(double pos, double power){
        leftVerSlide.setTargetPosition((int) pos);
        rightVerSlide.setTargetPosition((int) pos);

        leftVerSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightVerSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftVerSlide.setPower(power);
        rightVerSlide.setPower(power);
    }

    public void verticalSlide(double power){
        leftVerSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightVerSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftVerSlide.setPower(power);
        rightVerSlide.setPower(power);
    }

    public void resetVerticalSlide(){
        leftVerSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightVerSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftVerSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightVerSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void horizontalSlide(double pos, double power){
        leftHorSlide.setTargetPosition((int) pos);
        rightHorSlide.setTargetPosition((int) pos);

        leftHorSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightHorSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftHorSlide.setPower(power);
        rightHorSlide.setPower(power);
    }

    public void horizontalSlide(double power){
        leftHorSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightHorSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftHorSlide.setPower(power);
        rightHorSlide.setPower(power);
    }

    public void resetHorizontalSlide(){
        leftHorSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightHorSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftHorSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightHorSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setIntakeSubStatePos(double claw, double rotation, double arm){
        intakeRotationServo.setPosition(rotation);
        intakeArmServo.setPosition(arm);
        intakeClawServo.setPosition(claw);
    }

    public void setIntakeSubStatePos(double claw, double arm){
        intakeArmServo.setPosition(arm);
        intakeClawServo.setPosition(claw);
    }

    public void powerOffIntakeSubStatePos(){
        intakeRotationServo.getController().pwmDisable();
        intakeArmServo.getController().pwmDisable();
        intakeClawServo.getController().pwmDisable();
    }

    public void powerOnIntakeSubStatePos(){
        intakeRotationServo.getController().pwmEnable();
        intakeArmServo.getController().pwmEnable();
        intakeClawServo.getController().pwmEnable();
    }
    public void setOuttakeSubStatePos(double claw, double rotation, double wrist, double arm){
        outtakeRotationServo.setPosition(rotation);
        outtakeLeftArmServo.setPosition(arm);
        outtakeRightArmServo.setPosition(arm);
        outtakeClawServo.setPosition(claw);
        outtakeWristServo.setPosition(wrist);
    }

    public void powerOffOuttakeSubStatePos(){
        outtakeRotationServo.getController().pwmDisable();
        outtakeLeftArmServo.getController().pwmDisable();
        outtakeRightArmServo.getController().pwmDisable();
        outtakeClawServo.getController().pwmDisable();
        outtakeWristServo.getController().pwmDisable();
    }

    public void powerOnOuttakeSubStatePos(){
        outtakeRotationServo.getController().pwmEnable();
        outtakeLeftArmServo.getController().pwmEnable();
        outtakeRightArmServo.getController().pwmEnable();
        outtakeClawServo.getController().pwmEnable();
        outtakeWristServo.getController().pwmEnable();
    }

    public void setIntakeToTravelState() {
        setIntakeSubStatePos(
                InConstants.Claw_Open,
                InConstants.Wrist_Up,
                InConstants.Rotation_Straight
        );
        powerOnIntakeSubStatePos();
    }

    public void setIntakeToTransferState() {
        setIntakeSubStatePos(
                OutConstants.Claw_Close,
                OutConstants.Wrist_Up,
                OutConstants.Rotation_Straight
        );
        powerOnIntakeSubStatePos();
    }

    public void setOuttakeToTravelState() {
        setOuttakeSubStatePos(
                OutConstants.Claw_Open,
                OutConstants.Rotation_Straight,
                OutConstants.Wrist_Down,
                OutConstants.Arm_Parallel_Ground
        );
        powerOnOuttakeSubStatePos();
    }
}
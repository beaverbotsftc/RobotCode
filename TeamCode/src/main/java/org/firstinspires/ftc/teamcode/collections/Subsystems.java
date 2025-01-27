package org.firstinspires.ftc.teamcode.collections;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Subsystems {
    public DcMotor leftVerSlide = null;
    public DcMotor rightVerSlide = null;

    public void init(HardwareMap hardwareMap) {
        leftVerSlide = hardwareMap.get(DcMotor.class, "left ver slide");
        rightVerSlide = hardwareMap.get(DcMotor.class, "right ver slide");

        leftVerSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        rightVerSlide.setDirection(DcMotorSimple.Direction.FORWARD);

        leftVerSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftVerSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightVerSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightVerSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void verticalSlide(double power){
        leftVerSlide.setPower(power);
        rightVerSlide.setPower(power);
    }
}

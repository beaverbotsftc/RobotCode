package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
public class MotorController {
    public DcMotor drivenMotor;
    public float lsPosition = 0;

    public class DriveDistance {

    }

    public class DrivePosition {

    }

    public float lsGetPosition() {
        
        // gets the position from full retract to zero retract.
        return lsPosition; // Of type RetType
    }

    public float LSSetPosition() {
        //sets the position from getting the position.
        float lsStatus = 0;
        return lsStatus;
    }
}

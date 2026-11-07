package org.firstinspires.ftc.teamcode.experiments;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.beaver.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.beaver.command.HardwareManager;

@TeleOp
public class DistanceSensorExperiment extends CommandOpMode {
    private DistanceSensor distanceSensor;

    @Override
    public void onInit() {
        distanceSensor = HardwareManager.claim(DistanceSensor.class, "lidar");
    }

    @Override
    public void periodic() {
        double distance = distanceSensor.getDistance(DistanceUnit.INCH);
        if (distance < 100)
            addData("distance", distance);
    }
}

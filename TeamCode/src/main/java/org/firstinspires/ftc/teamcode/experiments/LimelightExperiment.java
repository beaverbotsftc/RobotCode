package org.firstinspires.ftc.teamcode.experiments;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.beaverbots.beaver.command.CommandRuntimeOpMode;
import org.beaverbots.beaver.command.HardwareManager;
import org.beaverbots.beaver.command.premade.Repeat;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.Gamepad;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group = "Experiments")
public class LimelightExperiment extends CommandRuntimeOpMode {
    private Gamepad gamepad;
    private Limelight3A limelight;

    @Override
    public void onInit() {
        gamepad = new Gamepad(gamepad1);
        limelight = HardwareManager.claim(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(9);
        limelight.start();

        register(gamepad);
    }

    @Override
    public void onStart() {
        schedule(new Repeat(() -> {
            LLResult result = limelight.getLatestResult();
            telemetry.addLine(String.valueOf(result.getFiducialResults().size()));
            if (result.getFiducialResults().isEmpty()) return;

            telemetry.addData("Robot pose", result.getBotpose().toString());
            telemetry.addData("Yaw", result.getFiducialResults().get(0).getTargetPoseRobotSpace().getOrientation().getYaw(AngleUnit.RADIANS));
            telemetry.addData("Pitch", result.getFiducialResults().get(0).getTargetPoseRobotSpace().getOrientation().getPitch(AngleUnit.RADIANS));
            telemetry.addData("Roll", result.getFiducialResults().get(0).getTargetPoseRobotSpace().getOrientation().getRoll(AngleUnit.RADIANS));
        }));
    }
}

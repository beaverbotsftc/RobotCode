package org.firstinspires.ftc.teamcode.teleop;

import android.provider.Settings;

import com.bylazar.field.PanelsField;
import com.qualcomm.hardware.andymark.AndyMarkTOF;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.seattlesolvers.solverslib.photon.PhotonCore;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.beaver.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.beaver.command.HardwareManager;
import org.firstinspires.ftc.teamcode.beaver.util.Stopwatch;
import org.firstinspires.ftc.teamcode.beaver.util.Transform;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrivetrain;

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class NASM extends CommandOpMode {
    private MecanumDrivetrain drivetrain;
    private DistanceSensor distanceSensor;
    private GoBildaPinpointDriver pinpoint;

    private List<Transform> collisions = new ArrayList<>();
    private List<Double> distances = new ArrayList<>();

    private Stopwatch stopwatch;

    private final double threshold = 200;

    private double maxDisplayed = 1000;

    @Override
    public void onInit() {
        PhotonCore.experimental.setMaximumParallelCommands(8);
        PhotonCore.enable();

        drivetrain = new MecanumDrivetrain();
        register(drivetrain);

        distanceSensor = HardwareManager.claim(DistanceSensor.class, "lidar");
        pinpoint = HardwareManager.claim(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetPosAndIMU();

        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setOffsets(-4, 2.5, DistanceUnit.INCH);

        stopwatch = new Stopwatch();
    }

    @Override
    public void onStart() {
        stopwatch.reset();
    }

    @Override
    public void periodic() {
        pinpoint.update();
        Transform position = new Transform(pinpoint.getPosition());

        double distance = distanceSensor.getDistance(DistanceUnit.INCH) + 8;
        if (distance < threshold) {
            Transform collision = position.lateral().add(Transform.BACKWARD.scale(distance).rotateLateral(position.theta));

            collisions.add(collision);
            distances.add(distance);
        }

        maxDisplayed += (gamepad.getRightTrigger() - gamepad.getLeftTrigger()) * stopwatch.getDt() * 500;
        if (maxDisplayed < 100) maxDisplayed = 100;

        addData("Distance", distance);
        addData("Position", position);
        addLine();
        addData("Max displayed", (int) maxDisplayed);
        addData("Datapoints", collisions.size());

        drivetrain.move(new Transform(gamepad.getLeftY(), -gamepad.getLeftX(), gamepad.getRightX()));

        PanelsField.INSTANCE.getField().setFill("black");
        PanelsField.INSTANCE.getField().moveCursor(-500, -500);
        PanelsField.INSTANCE.getField().rect(1000, 1000);

        PanelsField.INSTANCE.getField().setFill("white");

        double proportionDisplayed = maxDisplayed / collisions.size();
        if (proportionDisplayed > 1) proportionDisplayed = 1;

        for (int i = 0; i < Math.min(collisions.size(), maxDisplayed); i++) {
            int j = (int) (i / proportionDisplayed);
            PanelsField.INSTANCE.getField().moveCursor(collisions.get(j).x, collisions.get(j).y);
            PanelsField.INSTANCE.getField().circle(1);
        }

        PanelsField.INSTANCE.getField().setFill("red");
        PanelsField.INSTANCE.getField().moveCursor(position.x - 5, position.y - 5);
        PanelsField.INSTANCE.getField().rect(10, 10);
        Transform forward20 = position.add(Transform.FORWARD.scale(20).rotateLateral(position.theta));
        PanelsField.INSTANCE.getField().moveCursor(position.x, position.y);
        PanelsField.INSTANCE.getField().setCurrentOutlineFill("blue");
        PanelsField.INSTANCE.getField().setCurrentOutlineWidth(1);
        PanelsField.INSTANCE.getField().line(forward20.x, forward20.y);
        PanelsField.INSTANCE.getField().setCurrentOutlineWidth(0);

        PanelsField.INSTANCE.getField().update();
    }
}

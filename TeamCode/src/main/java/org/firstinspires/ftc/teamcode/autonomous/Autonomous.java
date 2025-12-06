package org.firstinspires.ftc.teamcode.autonomous;

import android.util.Pair;

import org.beaverbots.BeaverCommand.Command;
import org.beaverbots.BeaverCommand.CommandRuntimeOpMode;
import org.beaverbots.BeaverCommand.util.Instant;
import org.beaverbots.BeaverCommand.util.Parallel;
import org.beaverbots.BeaverCommand.util.RunUntil;
import org.beaverbots.BeaverCommand.util.Repeat;
import org.beaverbots.BeaverCommand.util.Sequential;
import org.beaverbots.BeaverCommand.util.Stopwatch;
import org.beaverbots.BeaverCommand.util.Wait;
import org.beaverbots.BeaverCommand.util.WaitUntil;
import org.beaverbots.beavertracking.HolonomicFollowPath;
import org.beaverbots.beavertracking.PIDF;
import org.beaverbots.beavertracking.PIDFAxis;
import org.beaverbots.beavertracking.Path;
import org.beaverbots.beavertracking.PathBuilder;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Motif;
import org.firstinspires.ftc.teamcode.Side;
import org.firstinspires.ftc.teamcode.subsystems.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Stopper;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainState;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.localizer.FusedLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Pinpoint;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class Autonomous extends CommandRuntimeOpMode {
    private Gamepad gamepad;
    private Drivetrain drivetrain;
    private Intake intake;
    private Shooter shooter;
    private Stopper stopper;
    private Pinpoint pinpoint;
    private Limelight limelight;
    private FusedLocalizer fusedLocalizer;

    private Side side;
    private Motif motif;

    @Override
    public void onInit() {
        gamepad = new Gamepad(gamepad1);
        drivetrain = new MecanumDrivetrain();
        intake = new Intake();
        shooter = new Shooter();
        stopper = new Stopper();
        pinpoint = new Pinpoint(new DrivetrainState(0, 0, 0));
        limelight = new Limelight();
        fusedLocalizer = new FusedLocalizer(pinpoint, limelight, new DrivetrainState(0, 0, 0));

        register(gamepad, drivetrain, intake, shooter, stopper, pinpoint, limelight, fusedLocalizer);
        limelight.goalPipeline();

        // Will be cancelled upon starting the opmode
        schedule(
                new Sequential(
                        new RunUntil(
                                new WaitUntil(() -> gamepad.getLeftStickPressed() || gamepad.getRightStickPressed()),
                                new Repeat(() -> {
                                    telemetry.addData("Position:", fusedLocalizer.getPosition());
                                    telemetry.addData("Covariance:", formatDoubleArray(fusedLocalizer.getCovariance().getData()));
                                }),
                                new Repeat(() -> {
                                    telemetry.addLine("Hello, Graisen! Press left if you are on the blue side, and right for red.");
                                    telemetry.addLine("Press up to add a new position, and down to remove the last one.");
                                    telemetry.addLine("Left and right bumper selects the left and right position respectively,");
                                    telemetry.addLine("and cross and triangle to cycle through the positions.");
                                    telemetry.addLine("Press down a stick to exit the configuration.");
                                    telemetry.addLine();
                                    telemetry.addLine("Current positions:");
                                    telemetry.addLine(IntStream.range(0, positions.size()).mapToObj(i -> {
                                        if (i == selectedPosition)
                                            return positions.get(i).toString().toUpperCase();
                                        else
                                            return positions.get(i).toString().toLowerCase();
                                    }).collect(Collectors.toList()).toString());

                                    if (gamepad.getDpadUpJustPressed()) {
                                        positions.add(Position.GPP);
                                    }

                                    if (gamepad.getDpadDownJustPressed() && !positions.isEmpty()) {
                                        positions.remove(positions.size() - 1);
                                    }

                                    if (gamepad.getCrossJustPressed()) {
                                        positions.set(selectedPosition, Position.values()[(positions.get(selectedPosition).ordinal() + 1) % Position.values().length]);
                                    }

                                    if (gamepad.getTriangleJustPressed()) {
                                        positions.set(selectedPosition, Position.values()[(positions.get(selectedPosition).ordinal() - 1 + Position.values().length) % Position.values().length]);
                                    }
                                })
                        ),
                        new Instant(() -> {
                            pinpoint.setPosition(fusedLocalizer.getPosition());
                            unregister(fusedLocalizer);
                            limelight.obeliskPipeline();
                        }),
                        new Repeat(() -> {
                            final Motif limelightMotif = limelight.getMotif(side);
                            if (limelightMotif != null) motif = limelightMotif;
                        })
                )
        );
    }

    @Override
    public void onStart() {
        cancelAll();
        List<Command> sequence = new ArrayList<>();
        DrivetrainState startPosition = pinpoint.getPosition();
        for (int i = 0; i < positions.size(); i++) {
            Position position = positions.get(i);

            switch (position) {
                case GPP:
                    Pair<Path, Path> path = new PathBuilder(startPosition.toList())
                            /*
                            .bezierTo(
                                    List.of(),
                                    List.of(),
                                    List.of()
                            )
                             */
                            .build();
                    startPosition = new DrivetrainState(path.second.position(0));
                    sequence.add(
                            followPathTemplate(
                                    path.first
                            )
                    );
                    break;
                case PGP:
                    break;
                case PPG:
                    break;
                case HUMAN:
                    break;
                case FAR_SHOOT:
                    break;
                case NEAR_SHOOT:
                    break;
                case PARK_END:
                    break;
                case NEAR_END:
                    break;
                case MID_END:
                    break;
                case FAR_END:
                    break;
            }
        }
        stopwatch.reset();
    }

    @Override
    public void periodic() {
        telemetry.addData("Position:", pinpoint.getPosition().toString());
    }

    private Command followPathTemplate(Path path) {
        return new Sequential(
                new HolonomicFollowPath(
                        path,
                        new PIDF(List.of(
                                new PIDFAxis(new PIDFAxis.K(Constants.pidPX, Constants.pidIX, Constants.pidDX, 1, 6, 48, Constants.pidTauX, Constants.pidGammaX)),
                                new PIDFAxis(new PIDFAxis.K(Constants.pidPY, Constants.pidIY, Constants.pidDY, 1, 6, 48, Constants.pidTauY, Constants.pidGammaY)),
                                new PIDFAxis(new PIDFAxis.K(Constants.pidPTheta, Constants.pidITheta, Constants.pidDTheta, 1, 6, 48, Constants.pidTauTheta, Constants.pidGammaTheta)))),
                        pinpoint, drivetrain),
                new Instant(() -> drivetrain.move(new DrivetrainState(0, 0, 0))));
    }

    public static String formatDoubleArray(double[][] array) {
        StringBuilder sb = new StringBuilder();

        for (double[] row : array) {
            for (int i = 0; i < row.length; i++) {
                String formatted = String.format("%.3f", row[i]);
                sb.append(formatted);

                if (i < row.length - 1) {
                    sb.append(" ");
                }
            }
            sb.append("\n");
        }

        return sb.toString();
    }
}

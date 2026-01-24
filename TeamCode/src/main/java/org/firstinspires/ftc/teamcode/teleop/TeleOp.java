package org.firstinspires.ftc.teamcode.teleop;

import org.beaverbots.beaver.command.CommandRuntimeOpMode;
import org.beaverbots.beaver.command.premade.Parallel;
import org.beaverbots.beaver.command.premade.router.Router;
import org.beaverbots.beaver.command.premade.router.Selector;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Side;
import org.firstinspires.ftc.teamcode.CrossModeStorage;
import org.firstinspires.ftc.teamcode.commands.AimAndResist;
import org.firstinspires.ftc.teamcode.commands.AimWhileDriving;
import org.firstinspires.ftc.teamcode.commands.DrivetrainControl;
import org.firstinspires.ftc.teamcode.commands.GoToBase;
import org.firstinspires.ftc.teamcode.commands.IntakeControl;
import org.firstinspires.ftc.teamcode.commands.ShooterControl;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensor;
import org.firstinspires.ftc.teamcode.subsystems.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Led;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Stopper;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainState;
import org.firstinspires.ftc.teamcode.subsystems.localizer.FusedLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Pinpoint;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends CommandRuntimeOpMode {
    private VoltageSensor voltageSensor;
    private Gamepad gamepad;
    private Drivetrain drivetrain;
    private Intake intake;
    private Stopper stopper;
    private Shooter shooter;
    private Pinpoint pinpoint;
    private ColorSensor colorSensor;
    private FusedLocalizer fusedLocalizer;
    private Limelight limelight;
    private Led led;

    @Override
    public void onInit() {
        voltageSensor = new VoltageSensor();
        gamepad = new Gamepad(gamepad1);
        drivetrain = new MecanumDrivetrain();
        intake = new Intake(voltageSensor);
        stopper = new Stopper();
        shooter = new Shooter(voltageSensor);
        pinpoint = new Pinpoint(CrossModeStorage.position);
        colorSensor = new ColorSensor();
        limelight = new Limelight();
        limelight.localizationPipeline();
        fusedLocalizer = new FusedLocalizer(pinpoint, limelight, CrossModeStorage.position, CrossModeStorage.covariance);
        led = new Led();
        register(voltageSensor, gamepad, drivetrain, intake, stopper, shooter, pinpoint, colorSensor, led, limelight, fusedLocalizer);
    }

    public void periodicInit() {
        if (gamepad.getCrossJustPressed()) {
            CrossModeStorage.side = CrossModeStorage.side == Side.RED ? Side.BLUE : Side.RED;
        }
        telemetry.addData("Side", CrossModeStorage.side);
    }

    @Override
    public void onStart() {
        schedule(
                /*new Router(
                        new Selector(() -> gamepad.getLeftStickPressed()),
                        new DrivetrainControl(drivetrain, gamepad),
                        new ShooterMode(pinpoint, drivetrain, CrossModeStorage.side, false)),
                new IntakeControl(intake, stopper, colorSensor, led, gamepad), shooterControl);
                        new ShooterMode(pinpoint, drivetrain, CrossModeStorage.side, false)
                ),*/
                new Router(
                        new Selector(() -> {
                            if (gamepad.getRightStickPressedToggle() && gamepad.getLeftStickPressed()) {
                                return 1;
                            }
                            if (gamepad.getLeftStickPressed()) {
                                return 2;
                            }
                            if (gamepad.getRightStickPressedToggle()) {
                                return 3;
                            }
                            if (gamepad.getGuide()) {
                                return 4;
                            }
                            return 0;
                        }
                        ),
                        new DrivetrainControl(drivetrain, gamepad),
                        new AimAndResist(fusedLocalizer, drivetrain, CrossModeStorage.side, true),
                        new AimAndResist(fusedLocalizer, drivetrain, CrossModeStorage.side, false),
                        new AimWhileDriving(fusedLocalizer, drivetrain, CrossModeStorage.side, gamepad),
                        new GoToBase(fusedLocalizer, drivetrain, CrossModeStorage.side)
                ),
                new Router(new Selector(() -> gamepad.getRightStickPressedToggle()),
                        new Parallel(
                                new ShooterControl(shooter, fusedLocalizer, false, CrossModeStorage.side, led, gamepad),
                                new IntakeControl(intake, stopper, fusedLocalizer, false, CrossModeStorage.side, colorSensor, led, gamepad)
                        ),
                        new Parallel(
                                new ShooterControl(shooter, fusedLocalizer, true, CrossModeStorage.side, led, gamepad),
                                new IntakeControl(intake, stopper, fusedLocalizer, true, CrossModeStorage.side, colorSensor, led, gamepad)
                        )
                )
        );
    }

    private FusedLocalizer relocalizer;

    @Override
    public void periodic() {


        telemetry.addData("Current RPM:", shooter.getVelocity());
        telemetry.addData("Distance to goal:", pinpoint.getPosition().lateralDistance(new DrivetrainState(Constants.GOAL_X, CrossModeStorage.side == Side.RED ? Constants.GOAL_Y : -Constants.GOAL_Y, 0)));

        telemetry.addData("X", fusedLocalizer.getPosition().getX());
        telemetry.addData("Y", fusedLocalizer.getPosition().getY());
        telemetry.addData("Theta", fusedLocalizer.getPosition().getTheta());

        telemetry.addData("X Var", fusedLocalizer.getCovariance().getEntry(0, 0));
        telemetry.addData("Y Var", fusedLocalizer.getCovariance().getEntry(1, 1));
        telemetry.addData("Theta Var", fusedLocalizer.getCovariance().getEntry(2, 2));

        CrossModeStorage.position = pinpoint.getPosition();

        telemetry.addData("Intake full", intake.full());
        telemetry.addData("Right trigger", gamepad.getRightTrigger());
        /*
        if (gamepad.getDpadUpJustPressed() && !a) {
            pinpoint.setPosition(fusedLocalizer.getPosition());
            a = true;
        }
         */

        /*
        if (gamepad.getDpadUpJustPressed()) {
            if (relocalizer == null) {
                limelight.localizationPipeline();
                relocalizer = new FusedLocalizer(pinpoint, limelight, pinpoint.getPosition());
                register(relocalizer);
            } else {
                pinpoint.setPosition(relocalizer.getPosition());
                unregister(relocalizer);
                relocalizer = null;
                shooter.hardStopSetting = false;
            }
        }

         */

        if (gamepad.getDpadRightPressedToggle()) {
            telemetry.addLine("Limelight DISABLED.");
            fusedLocalizer.disableLimelight();
        } else {
            fusedLocalizer.enableLimelight();
        }
        /*
        if (gamepad.getDpadUpPressedToggle()) {
            telemetry.addData("X", relocalizer.getPosition().getX());
            telemetry.addData("Y", relocalizer.getPosition().getY());
            telemetry.addData("Theta", relocalizer.getPosition().getTheta());

            telemetry.addData("X Var", relocalizer.getCovariance().getEntry(0, 0));
            telemetry.addData("Y Var", relocalizer.getCovariance().getEntry(1, 1));
            telemetry.addData("Theta Var", relocalizer.getCovariance().getEntry(2, 2));
            
            shooter.hardStopSetting = true;
        }
        if (gamepad.getDpadRightJustPressed() && relocalizer != null) {
            unregister(relocalizer);
            relocalizer = null;
            gamepad.setDpadUpPressedToggle(false);
            shooter.hardStopSetting = false;
        }
         */
    }
}

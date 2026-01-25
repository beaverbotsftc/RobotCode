package org.firstinspires.ftc.teamcode.experiments;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="Turret Tester", group="Examples")
public class TurretExperiment extends LinearOpMode {

    private DcMotorEx motor;
    private DcMotorEx motor2;
    private DcMotor intake;
    private DcMotor stopper;
    private Servo hoodServo;
    private Servo turret;
    private Servo turret1;
    private double power = 0;
    private double turretPos = 0.5;
    private static final double MAX_RPM = 6000.0;
    private static final int TICKS_PER_REV = 28;

    @Override
    public void runOpMode(){
        motor = hardwareMap.get(DcMotorEx.class, "shoot");
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor2 = hardwareMap.get(DcMotorEx.class, "shoot2");
        motor2.setDirection(DcMotor.Direction.REVERSE);

        hoodServo = hardwareMap.get(Servo.class, "hood");
        turret = hardwareMap.get(Servo.class, "turret servo");
        turret1 = hardwareMap.get(Servo.class, "turret servo2");

        turret.setDirection(Servo.Direction.FORWARD);
        turret1.setDirection(Servo.Direction.FORWARD);

        // Reset encoders
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        stopper = hardwareMap.get(DcMotorEx.class, "stopper");

        waitForStart();

        while (opModeIsActive()) {
            double speed = -gamepad1.left_stick_y;

            if(gamepad1.triangleWasPressed()){
                turretPos += 0.025;
            }else if (gamepad1.crossWasPressed()){
                turretPos -= 0.025;
            }

            turret.setPosition(turretPos);
            turret1.setPosition(turretPos);

            // Increment power when triangle is pressed
            if (gamepad1.rightBumperWasPressed()) {
                if (power < 1) {
                    power += 0.05;
                }
            }

            // Decrement power when X is pressed
            if (gamepad1.leftBumperWasPressed()) {
                if(power > 0){
                    power -= 0.05;
                }
            }

            if(gamepad1.dpadLeftWasPressed()){
                hoodServo.setPosition(hoodServo.getPosition() + 0.05);
            }else if(gamepad1.dpadRightWasPressed()){
                hoodServo.setPosition(hoodServo.getPosition() - 0.05);
            }

            // Apply motor power
            double outputPower = power * speed;
            motor.setPower(outputPower);
            motor2.setPower(outputPower);

            intake.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
            stopper.setPower(gamepad1.right_stick_y);

            // Calculate RPMs
            double rpm1 = ticksPerSecondToRPM(motor.getVelocity());
            double rpm2 = ticksPerSecondToRPM(motor2.getVelocity());
            double encoder = motor.getCurrentPosition();
            double encoder2 = motor2.getCurrentPosition();

            // Telemetry
            telemetry.addData("Joystick Value", "%.2f", gamepad1.left_stick_y);
            telemetry.addData("Motor Power Limit", "%.2f", power);
            telemetry.addData("Motor 1 RPM", "%.1f", rpm1);
            telemetry.addData("Motor 2 RPM", "%.1f", rpm2);
            telemetry.addData("Motor 1 encode", "%.1f", encoder);
            telemetry.addData("Motor 2 encode", "%.1f", encoder2);

            telemetry.addLine("");
            telemetry.addData("Hood Pos", hoodServo.getPosition());
            telemetry.addData("Turret Pos", turretPos);
            telemetry.update();
        }
    }

    private double ticksPerSecondToRPM(double ticksPerSecond) {
        return (ticksPerSecond / TICKS_PER_REV) * 60.0;
    }
}

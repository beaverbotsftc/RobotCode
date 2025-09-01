package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.collections.InConstants;
import org.firstinspires.ftc.teamcode.collections.Motors;
import org.firstinspires.ftc.teamcode.collections.OutConstants;
import org.firstinspires.ftc.teamcode.collections.Subsystems;

@TeleOp(name="Tuning TeleOp", group="Linear OpMode")
public class TuningTeleOp extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private Motors motors = new Motors();
    private Subsystems subsystems = new Subsystems();
    private InConstants in = new InConstants();
    private OutConstants out = new OutConstants();
    private double speed = 0.5;
    private double turnspeed = 0.5;
    private double inverse = 1;
    private boolean isRightOuttakeArm = true;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        motors.init(hardwareMap);
        subsystems.init(hardwareMap);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            if (gamepad1.left_bumper){
                speed = 0.25;
                turnspeed = 0.15;
            } else if (gamepad1.right_bumper) {
                speed = 1.0;
                turnspeed = 0.6;
            }else{
                speed = 0.55;
                turnspeed = 0.6;
            }


            if(gamepad1.right_trigger > 0.05) {
                subsystems.verticalSlide(gamepad1.right_trigger);
            }else if(gamepad1.left_trigger > 0.05){
                subsystems.verticalSlide(-gamepad1.left_trigger);
            }else{
                subsystems.verticalSlide(0);
            }

            subsystems.horizontalSlide(gamepad2.left_stick_y);

            if (gamepad1.dpad_up){
                subsystems.intakeClawServo.setPosition(in.Claw_Open);
            }else if(gamepad1.dpad_down){
                subsystems.intakeClawServo.setPosition(in.Claw_Close);
            }

            if (gamepad1.dpad_right){
                subsystems.intakeRotationServo.setPosition(in.Rotation_Flipped);
            }else if(gamepad1.dpad_left){
                subsystems.intakeRotationServo.setPosition(in.Rotation_Straight);
            }

            if (gamepad1.triangle){
                subsystems.intakeArmServo.setPosition(in.Wrist_Down);
            }else if(gamepad1.cross){
                subsystems.intakeArmServo.setPosition(in.Wrist_Up);
            }else if(gamepad1.square){
                subsystems.intakeArmServo.setPosition(in.Wrist_Straight);
            }

            if(gamepad2.share){
                isRightOuttakeArm = false;
            } else if (gamepad2.options) {
                isRightOuttakeArm = true;
            }

            if (gamepad2.left_bumper) {
                if (isRightOuttakeArm){
                    subsystems.outtakeRightArmServo.setPosition(out.Arm_Parallel_Ground);
                }else{
                    subsystems.outtakeLeftArmServo.setPosition(out.Arm_Parallel_Slides);
                }
            } else if (gamepad2.right_bumper) {
                if (isRightOuttakeArm){
                    subsystems.outtakeRightArmServo.setPosition(out.Arm_Parallel_Ground);
                }else{
                    subsystems.outtakeLeftArmServo.setPosition(out.Arm_Parallel_Slides);
                }
            }

            if (gamepad2.dpad_up){
                subsystems.outtakeClawServo.setPosition(out.Claw_Close);
            }else if(gamepad2.dpad_down){
                subsystems.outtakeClawServo.setPosition(out.Claw_Open);
            }

            if (gamepad2.dpad_right){
                subsystems.outtakeRotationServo.setPosition(out.Rotation_Flipped);
            } else if (gamepad2.dpad_left){
                subsystems.outtakeRotationServo.setPosition(out.Rotation_Straight);
            }

            if (gamepad2.triangle){
                subsystems.outtakeWristServo.setPosition(out.Wrist_Up);
            } else if (gamepad2.cross){
                subsystems.outtakeWristServo.setPosition(out.Wrist_Down);
            }
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   =  -gamepad1.left_stick_y * speed * inverse;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x * speed * inverse;
            double yaw     =  gamepad1.right_stick_x * turnspeed;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }
            
            motors.leftFrontDrive.setPower(leftFrontPower);
            motors.rightFrontDrive.setPower(rightFrontPower);
            motors.leftBackDrive.setPower(leftBackPower);
            motors.rightBackDrive.setPower(rightBackPower);

            // --- Robot Status ---
            telemetry.addLine("--- ROBOT STATUS ---");
            telemetry.addData("Runtime", runtime.toString());
            telemetry.addData("Speed Mode", (gamepad1.left_bumper ? "SLOW" : (gamepad1.right_bumper ? "FAST" : "NORMAL")));
            telemetry.addData("Selected Outtake Arm", isRightOuttakeArm ? "RIGHT" : "LEFT");
            telemetry.addLine(); // Blank line for spacing

            // --- Motor & Servo Data ---
            telemetry.addLine("--- DATA ---");
            telemetry.addData("Wheel Power", "FL:%.2f, FR:%.2f, BL:%.2f, BR:%.2f", leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
            telemetry.addData("Vertical Slide Power", "%.2f", gamepad1.right_trigger - gamepad1.left_trigger);
            telemetry.addData("Horizontal Slide Power", "%.2f", gamepad2.left_stick_y);
            telemetry.addData("Intake Servos (Claw, Rot, Arm)", "%.2f, %.2f, %.2f",
                    subsystems.intakeClawServo.getPosition(),
                    subsystems.intakeRotationServo.getPosition(),
                    subsystems.intakeArmServo.getPosition());
            telemetry.addData("Outtake Servos (Claw, Rot, Wrist)", "%.2f, %.2f, %.2f",
                    subsystems.outtakeClawServo.getPosition(),
                    subsystems.outtakeRotationServo.getPosition(),
                    subsystems.outtakeWristServo.getPosition());
            telemetry.addLine();

            // --- Controls Guide ---
            telemetry.addLine("--- CONTROLS ---");
            telemetry.addLine("--- GP1: Drive ---");
            telemetry.addData("L/R Bumper", "Speed SLOW / FAST");
            telemetry.addData("L/R Trigger", "Vertical Slide DOWN / UP");
            telemetry.addData("D-Pad U/D", "Intake Claw OPEN / CLOSE");
            telemetry.addData("D-Pad L/R", "Intake Rot STRAIGHT / FLIPPED");
            telemetry.addData("Triangle/Cross/Square", "Intake Arm DOWN / UP / STRAIGHT");
            telemetry.addLine();

            telemetry.addLine("--- GP2: Operator ---");
            telemetry.addData("Share/Options", "Select LEFT / RIGHT Arm");
            telemetry.addData("Left Stick Y", "Horizontal Slide");
            telemetry.addData("L/R Bumper", "Outtake Arm to Position");
            telemetry.addData("D-Pad U/D", "Outtake Claw CLOSE / OPEN");
            telemetry.addData("D-Pad L/R", "Outtake Rot STRAIGHT / FLIPPED");
            telemetry.addData("Triangle/Cross", "Outtake Wrist UP / DOWN");

            telemetry.update();
        }
    }

}

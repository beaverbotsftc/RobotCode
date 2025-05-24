package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.collections.Motors;
import org.firstinspires.ftc.teamcode.collections.Subsystems;

@TeleOp(name="Basic TeleOp", group="Linear OpMode")
public class BasicTeleOp extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private Motors motors = new Motors();
    private Subsystems subsystems = new Subsystems();
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
                subsystems.intakeClawServo.setPosition(subsystems.intakeClawServo.getPosition() + 0.001);
            }else if(gamepad1.dpad_down){
                subsystems.intakeClawServo.setPosition(subsystems.intakeClawServo.getPosition() - 0.001);
            }

            if (gamepad1.dpad_right){
                subsystems.intakeRotationServo.setPosition(subsystems.intakeRotationServo.getPosition() + 0.001);
            }else if(gamepad1.dpad_left){
                subsystems.intakeRotationServo.setPosition(subsystems.intakeRotationServo.getPosition() - 0.001);
            }

            if (gamepad1.triangle){
                subsystems.intakeArmServo.setPosition(subsystems.intakeArmServo.getPosition() + 0.001);
            }else if(gamepad1.cross){
                subsystems.intakeArmServo.setPosition(subsystems.intakeArmServo.getPosition() - 0.001);
            }

            if(gamepad2.share){
                isRightOuttakeArm = false;
            } else if (gamepad2.options) {
                isRightOuttakeArm = true;
            }

            if (gamepad2.left_bumper) {
                if (isRightOuttakeArm){
                    subsystems.outtakeRightArmServo.setPosition(subsystems.outtakeRightArmServo.getPosition() + 0.001);
                }else{
                    subsystems.outtakeLeftArmServo.setPosition(subsystems.outtakeLeftArmServo.getPosition() + 0.001);
                }
            } else if (gamepad2.right_bumper) {
                if (isRightOuttakeArm){
                    subsystems.outtakeRightArmServo.setPosition(subsystems.outtakeRightArmServo.getPosition() - 0.001);
                }else{
                    subsystems.outtakeLeftArmServo.setPosition(subsystems.outtakeLeftArmServo.getPosition() - 0.001);
                }
            }

            if (gamepad2.dpad_up){
                subsystems.outtakeClawServo.setPosition(subsystems.outtakeClawServo.getPosition() + 0.001);
            }else if(gamepad2.dpad_down){
                subsystems.outtakeClawServo.setPosition(subsystems.outtakeClawServo.getPosition() - 0.001);
            }

            if (gamepad1.dpad_right){
                subsystems.outtakeRotationServo.setPosition(subsystems.outtakeRotationServo.getPosition() + 0.001);
            } else if (gamepad1.dpad_left){
                subsystems.outtakeRotationServo.setPosition(subsystems.outtakeRotationServo.getPosition() - 0.001);
            }

            if (gamepad1.triangle){
                subsystems.outtakeWristServo.setPosition(subsystems.outtakeWristServo.getPosition() + 0.001);
            } else if (gamepad1.cross){
                subsystems.outtakeWristServo.setPosition(subsystems.outtakeWristServo.getPosition() - 0.001);
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

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Horizontal Slide Power", subsystems.leftHorSlide.getPower());
            telemetry.addData("Intake Claw Servo Position", subsystems.intakeClawServo.getPosition());
            telemetry.addData("Intake Rotation Servo Position", subsystems.intakeRotationServo.getPosition());
            // ------------ User Instructions Telemetry ------------
            telemetry.addLine("--- Gamepad 1 Controls ---");
            telemetry.addData("Left Stick (Y, X)", "Drive Fwd/Rev, Strafe L/R");
            telemetry.addData("Right Stick (X)", "Turn L/R");
            telemetry.addData("Left Bumper", "Hold for SLOW Speed Mode");
            telemetry.addData("Right Bumper", "Hold for FAST Speed Mode");
            telemetry.addData("Right Trigger", "Vertical Slide UP (Power: %.2f)", gamepad1.right_trigger);
            telemetry.addData("Left Trigger", "Vertical Slide DOWN (Power: %.2f)", gamepad1.left_trigger);
            telemetry.addData("D-Pad Up", "Intake Claw: Adjust Pos (+0.001)");
            telemetry.addData("D-Pad Down", "Intake Claw: Adjust Pos (-0.001)");
            telemetry.addData("D-Pad Right", "Intake Rotate & Outtake Rotate: Adjust Pos (+0.001)");
            telemetry.addData("D-Pad Left", "Intake Rotate & Outtake Rotate: Adjust Pos (-0.001)");
            telemetry.addData("Triangle (Y)", "Intake Arm & Outtake Wrist: Adjust Pos (+0.001)");
            telemetry.addData("Cross (A)", "Intake Arm & Outtake Wrist: Adjust Pos (-0.001)");

            telemetry.addLine("--- Gamepad 2 Controls ---");
            telemetry.addData("Left Stick (Y)", "Horizontal Slide Fwd/Rev (Power: %.2f)", gamepad2.left_stick_y);
            telemetry.addData("Share Button (View/Back)", "Select LEFT Outtake Arm");
            telemetry.addData("Options Button (Menu/Start)", "Select RIGHT Outtake Arm");
            telemetry.addData("  Currently Selected Arm", isRightOuttakeArm ? "RIGHT" : "LEFT");
            telemetry.addData("Left Bumper", "Selected Outtake Arm: Adjust Pos (+0.001)");
            telemetry.addData("Right Bumper", "Selected Outtake Arm: Adjust Pos (-0.001)");
            telemetry.addData("D-Pad Up", "Outtake Claw: Adjust Pos (+0.001)");
            telemetry.addData("D-Pad Down", "Outtake Claw: Adjust Pos (-0.001)");
            telemetry.addLine("------------------------------------"); // Separator before other data
            // ------------ End User Instructions Telemetry ------------
            telemetry.update();
        }
    }

}

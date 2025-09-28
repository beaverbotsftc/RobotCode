package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.collections.InConstants;
import org.firstinspires.ftc.teamcode.collections.Motors;
import org.firstinspires.ftc.teamcode.collections.SubsystemsV2;


@TeleOp(name="SemiAdvanced TeleOp", group="Linear OpMode")
public class SemiAdvancedTeleOp extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Motors motors = new Motors();
    private SubsystemsV2 sub = new SubsystemsV2();

    // Control variables
    private double speed = 0.5;
    private double turnspeed = 0.5;
    private boolean isHorSlideEnabled = false;
    private boolean isIntakeRotationEnabled = false;

    /**
     * This enum defines the simplified state machine for the intake process only.
     * The robot will cycle through these states to pick up a sample.
     */
    public enum SampleStates {
        TRAVEL,             // Intake is stowed, robot is free to move
        READY_TO_EXTEND,    // Intake is lowered, ready for driver to extend slides
        PICK_UP,            // Intake is in position to grab the sample
        PICKED_UP,          // Claw is closed on the sample
        RETRACTED;          // Slides and arm are brought back to a safe position

        public SampleStates increment() {
            return SampleStates.values()[(this.ordinal() + 1) % SampleStates.values().length];
        }

        public SampleStates decrement(){
            return SampleStates.values()[(this.ordinal() + SampleStates.values().length - 1) % SampleStates.values().length];
        }
    }

    private SampleStates currentSampleState = SampleStates.TRAVEL;


    @Override
    public void runOpMode() {
        motors.init(hardwareMap);
        sub.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // Set initial robot state
        stateMachines();

        while (opModeIsActive()) {

            // --- Drivetrain Logic ---
            // This code was moved here from the handleDriving() method.
            double axial   = -changeInput(gamepad1.left_stick_y) * speed;
            double lateral =  changeInput(gamepad1.left_stick_x) * speed;
            double yaw     =  changeInput(gamepad1.right_stick_x) * turnspeed;

            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            // Normalize the motor powers
            double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
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

            // --- Other Controls ---
            speedControls();

            // Manual Intake Slide Control (only active in certain states)
            if(isHorSlideEnabled){
                if(gamepad1.left_trigger > 0.05) {
                    sub.horizontalSlide(-changeInput(gamepad1.left_trigger));
                } else if(gamepad1.right_trigger > 0.05){
                    sub.horizontalSlide(changeInput(gamepad1.right_trigger));
                } else {
                    sub.horizontalSlide(0);
                }
            }

            // State Machine Progression
            handleStateProgression();

            // --- TELEMETRY --- //
            updateTelemetry();
        }
    }

    /**
     * Manages the robot's state machine for the intake process.
     * This method is called whenever the state changes.
     */
    private void stateMachines() {
        switch (currentSampleState) {
            case TRAVEL:
                // Intake is stowed safely for driving. Manual slide control is off.
                setControlFlags(true, false);
                sub.setIntakeToTravelState();
                sub.powerOnIntakeSubStatePos();
                break;
            case READY_TO_EXTEND:
                // Intake is ready to extend. Driver can now control horizontal slides.
                setControlFlags(true, true);
                sub.setIntakeSubStatePos(
                        InConstants.Claw_Open,
                        InConstants.Rotation_Straight,
                        InConstants.Wrist_Straight
                );
                sub.powerOnIntakeSubStatePos();
                break;
            case PICK_UP:
                // Intake is lowered to the floor to pick up. Driver control remains.
                setControlFlags(true, true);
                sub.setIntakeSubStatePos(
                        InConstants.Claw_Open,
                        InConstants.Wrist_Down
                );
                sub.powerOnIntakeSubStatePos();
                break;
            case PICKED_UP:
                // Claw closes on the sample. Manual slide control is disabled.
                setControlFlags(true, true);
                sub.setIntakeSubStatePos(
                        InConstants.Claw_Close,
                        InConstants.Wrist_Down
                );
                sub.powerOnIntakeSubStatePos();
                break;
            case RETRACTED:
                // Intake wrist moves up to carry the sample securely.
                setControlFlags(true, false);
                sub.setIntakeSubStatePos(
                        InConstants.Claw_Close,
                        InConstants.Rotation_Straight,
                        InConstants.Wrist_Up
                );
                sub.powerOnIntakeSubStatePos();
                break;
        }
    }

    /**
     * Checks for button presses to advance or reverse the state machine.
     * Triangle (Y) increments the state, Square (X) decrements it.
     */
    private void handleStateProgression() {
        // Check Triangle button (gamepad1.y) for incrementing the state
        if (gamepad1.squareWasPressed()) {
            currentSampleState = currentSampleState.increment();
            stateMachines();
        }
        // Check Square button (gamepad1.x) for decrementing the state
        if (gamepad1.crossWasPressed()) {
            currentSampleState = currentSampleState.decrement();
            stateMachines();
        }
        //change wrist if its enabled5
        if (gamepad1.triangleWasPressed() && isIntakeRotationEnabled){
            sub.flipIntakeRotation();
        }
    }

    /**
     * Updates the driver station telemetry.
     */
    private void updateTelemetry() {
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("--- STATE ---", "Current State: " + currentSampleState.toString());
        telemetry.addData("Horizontal Slide Control Enabled", isHorSlideEnabled);
        telemetry.addData("--- Intake ---", "");
        telemetry.addData(" Intake Claw", sub.intakeClawServo.getPosition());
        telemetry.addData(" Intake Wrist", sub.intakeRotationServo.getPosition());
        telemetry.addData(" Intake Arm", sub.intakeArmServo.getPosition());
        telemetry.addData("--- Slides ---", "");
        telemetry.addData(" LeftHor Pos", sub.leftHorSlide.getCurrentPosition());
        telemetry.update();
    }

    /**
     * Sets the speed multipliers for driving.
     */
    private void speedControls() {
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
    }

    /**
     * Sets the boolean flags that enable/disable manual control of subsystems.
     */
    private void setControlFlags(boolean hor, boolean rotation){
        isHorSlideEnabled = hor;
        isIntakeRotationEnabled = rotation;
    }

    /**
     * Applies a curve to the joystick input for finer control at low speeds.
     */
    private double changeInput(double x){
        return Math.signum(x) * (1 - Math.cos(x * Math.PI / 2.0));
    }


}
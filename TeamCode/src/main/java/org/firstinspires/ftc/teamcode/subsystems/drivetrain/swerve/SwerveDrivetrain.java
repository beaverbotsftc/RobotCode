package org.firstinspires.ftc.teamcode.subsystems.drivetrain.swerve;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.beaverbots.beaver.InfiniteServo;
import org.beaverbots.beaver.optimizedhardware.OptimizedCRServo;
import org.beaverbots.beaver.optimizedhardware.OptimizedMotor;
import org.beaverbots.beaver.command.CommandOpMode;
import org.beaverbots.beaver.command.HardwareManager;
import org.beaverbots.beaver.pidf.PIDFAxis;
import org.beaverbots.beaver.util.Transform;
import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;

import java.util.List;

public class SwerveDrivetrain implements Drivetrain {
    private SwerveModule frontLeftModule;
    private SwerveModule frontRightModule;
    private SwerveModule backLeftModule;
    private SwerveModule backRightModule;

    private final VoltageSensor voltageSensor;

    private final boolean maxCapability;

    private static final Transform frontLeftPosition = new Transform(1, 1);
    private static final Transform frontRightPosition = new Transform(1, -1);
    private static final Transform backLeftPosition = new Transform(-1, 1);
    private static final Transform backRightPosition = new Transform(-1, -1);

    private static final double NOMINAL_VOLTAGE = 12;

    public SwerveDrivetrain(VoltageSensor voltageSensor, boolean maxCapability) {
        this.voltageSensor = voltageSensor;
        this.maxCapability = maxCapability;

        OptimizedCRServo frontLeftServo = new OptimizedCRServo(HardwareManager.claim(CRServo.class, "front left servo"), 0);
        frontLeftServo.setPwmRange(500, 2500);
        AnalogInput frontLeftEncoder = HardwareManager.get(AnalogInput.class, "front left servo encoder");
        InfiniteServo frontLeftInfiniteServo = new InfiniteServo(
                frontLeftServo,
                frontLeftEncoder,
                new PIDFAxis(
                        new PIDFAxis.K(
                                //0.2455, 0.1564, 0.0048, new double[]{0}, 1, 1, 0.2281, 7.8312
                                //0.3081, 0.7168, 0.0017, new double[]{0}, 1, 1, 0.7728, 613
                                //                        0.2100, 0.5418, 0.0023, new double[]{0}, 1, 1, 0.6119, 245
                                0.2043, 0.3854, 0.0118, new double[]{0}, 1, 1, 0.9731, 988, 0.1
                        )
                ),
                4.4915
        );
        OptimizedMotor frontLeftMotor = new OptimizedMotor(HardwareManager.claim(DcMotorEx.class, "front left drive"), 0);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftModule = new SwerveModule(frontLeftInfiniteServo, frontLeftMotor, frontLeftPosition);

        OptimizedCRServo frontRightServo = new OptimizedCRServo(HardwareManager.claim(CRServo.class, "front right servo"), 0);
        frontRightServo.setPwmRange(500, 2500);
        AnalogInput frontRightEncoder = HardwareManager.get(AnalogInput.class, "front right servo encoder");
        InfiniteServo frontRightInfiniteServo = new InfiniteServo(
                frontRightServo,
                frontRightEncoder,
                new PIDFAxis(
                        new PIDFAxis.K(
                                //0.2455, 0.1564, 0.0048, new double[]{0}, 1, 1, 0.2281, 7.8312
                                //0.3081, 0.7168, 0.0017, new double[]{0}, 1, 1, 0.7728, 613
                                //                        0.2100, 0.5418, 0.0023, new double[]{0}, 1, 1, 0.6119, 245
                                0.2043, 0.3854, 0.0118, new double[]{0}, 1, 1, 0.9731, 988, 0.1
                        )
                ),
                3.2768
        );
        OptimizedMotor frontRightMotor = new OptimizedMotor(HardwareManager.claim(DcMotorEx.class, "front right drive"), 0);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightModule = new SwerveModule(frontRightInfiniteServo, frontRightMotor, frontRightPosition);

        OptimizedCRServo backLeftServo = new OptimizedCRServo(HardwareManager.claim(CRServo.class, "back left servo"), 0);
        backLeftServo.setPwmRange(500, 2500);
        AnalogInput backLeftEncoder = HardwareManager.get(AnalogInput.class, "back left servo encoder");
        InfiniteServo backLeftInfiniteServo = new InfiniteServo(
                backLeftServo,
                backLeftEncoder,
                new PIDFAxis(
                        new PIDFAxis.K(
                                //0.2455, 0.1564, 0.0048, new double[]{0}, 1, 1, 0.2281, 7.8312
                                //0.3081, 0.7168, 0.0017, new double[]{0}, 1, 1, 0.7728, 613
                                //0.2100, 0.5418, 0.0023, new double[]{0}, 1, 1, 0.6119, 245
                                0.2043, 0.3854, 0.0118, new double[]{0}, 1, 1, 0.9731, 988, 0.1
                        )
                ),
                2.0735
        );
        OptimizedMotor backLeftMotor = new OptimizedMotor(HardwareManager.claim(DcMotorEx.class, "back left drive"), 0);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftModule = new SwerveModule(backLeftInfiniteServo, backLeftMotor, backLeftPosition);

        OptimizedCRServo backRightServo = new OptimizedCRServo(HardwareManager.claim(CRServo.class, "back right servo"), 0);
        backRightServo.setPwmRange(500, 2500);
        AnalogInput backRightEncoder = HardwareManager.get(AnalogInput.class, "back right servo encoder");
        InfiniteServo backRightInfiniteServo = new InfiniteServo(
                backRightServo,
                backRightEncoder,
                new PIDFAxis(
                        new PIDFAxis.K(
                                //0.2455, 0.1564, 0.0048, new double[]{0}, 1, 1, 0.2281, 7.8312
                                //0.3081, 0.7168, 0.0017, new double[]{0}, 1, 1, 0.7728, 613
                                //                        0.2100, 0.5418, 0.0023, new double[]{0}, 1, 1, 0.6119, 245
                                0.2043, 0.3854, 0.0118, new double[]{0}, 1, 1, 0.9731, 988, 0.1
                        )
                ),
                1.4813
        );
        OptimizedMotor backRightMotor = new OptimizedMotor(HardwareManager.claim(DcMotorEx.class, "back right drive"), 0);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightModule = new SwerveModule(backRightInfiniteServo, backRightMotor, backRightPosition);
    }

    public SwerveDrivetrain(VoltageSensor voltageSensor) {
        this(voltageSensor, true);
    }

    public void move(Transform force) {
        double scalar = maxCapability ? 1 : NOMINAL_VOLTAGE / voltageSensor.getVoltage();
        Transform scaledForce = force.scale(scalar);
        frontLeftModule.drive(scaledForce);
        frontRightModule.drive(scaledForce);
        backLeftModule.drive(scaledForce);
        backRightModule.drive(scaledForce);
    }

    public void preempt(Transform force) {
        frontLeftModule.turn(force);
        frontRightModule.turn(force);
        backLeftModule.turn(force);
        backRightModule.turn(force);
    }

    public void x() {
        frontLeftModule.setAngle(Transform.ZERO.angleTo(frontLeftPosition));
        frontRightModule.setAngle(Transform.ZERO.angleTo(frontRightPosition));
        backLeftModule.setAngle(Transform.ZERO.angleTo(backLeftPosition));
        backRightModule.setAngle(Transform.ZERO.angleTo(backRightPosition));
    }

    public void move(Transform force, Transform position) {
        move(force.toLocalVelocity(position));
    }

    public void move(List<Double> force, List<Double> position) {
        move(new Transform(force), new Transform(position));
    }

    public void periodic() {
        frontLeftModule.periodic();
        frontRightModule.periodic();
        backLeftModule.periodic();
        backRightModule.periodic();

        CommandOpMode.addData("Front Left Position", frontLeftModule.getAngle());
        CommandOpMode.addData("Front Right Position", frontRightModule.getAngle());
        CommandOpMode.addData("Back Left Position", backLeftModule.getAngle());
        CommandOpMode.addData("Back Right Position", backRightModule.getAngle());
    }
}
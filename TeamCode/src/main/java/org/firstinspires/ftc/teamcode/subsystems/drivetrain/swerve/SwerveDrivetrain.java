package org.firstinspires.ftc.teamcode.subsystems.drivetrain.swerve;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.beaverbots.beaver.InfiniteServo;
import org.beaverbots.beaver.cachedhardware.CachedCRServo;
import org.beaverbots.beaver.cachedhardware.CachedMotor;
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

        CachedCRServo frontLeftServo = new CachedCRServo(HardwareManager.claim(CRServo.class, "front left servo"), 0);
        frontLeftServo.setPwmRange(500, 2500);
        AnalogInput frontLeftEncoder = HardwareManager.get(AnalogInput.class, "front left servo encoder");
        InfiniteServo frontLeftInfiniteServo = new InfiniteServo(
                frontLeftServo,
                frontLeftEncoder,
                new PIDFAxis(
                        new PIDFAxis.K(
                                //0.2455, 0.1564, 0.0048, new double[]{0}, 1, 1, 0.2281, 7.8312
                                0.3081, 0.7168, 0.0017, new double[]{0}, 1, 1, 0.7728, 613
                        )
                ),
                3.175864573447136
        );
        CachedMotor frontLeftMotor = new CachedMotor(HardwareManager.claim(DcMotorEx.class, "front left drive"), 0);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftModule = new SwerveModule(frontLeftInfiniteServo, frontLeftMotor, voltageSensor, frontLeftPosition);

        CachedCRServo frontRightServo = new CachedCRServo(HardwareManager.claim(CRServo.class, "front right servo"), 0);
        frontRightServo.setPwmRange(500, 2500);
        AnalogInput frontRightEncoder = HardwareManager.get(AnalogInput.class, "front right servo encoder");
        InfiniteServo frontRightInfiniteServo = new InfiniteServo(
                frontRightServo,
                frontRightEncoder,
                new PIDFAxis(
                        new PIDFAxis.K(
                                //0.2455, 0.1564, 0.0048, new double[]{0}, 1, 1, 0.2281, 7.8312
                                0.3081, 0.7168, 0.0017, new double[]{0}, 1, 1, 0.7728, 613
                        )
                ),
                3.2805843285667957
        );
        CachedMotor frontRightMotor = new CachedMotor(HardwareManager.claim(DcMotorEx.class, "front right drive"), 0);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightModule = new SwerveModule(frontRightInfiniteServo, frontRightMotor, voltageSensor, frontRightPosition);

        CachedCRServo backLeftServo = new CachedCRServo(HardwareManager.claim(CRServo.class, "back left servo"), 0);
        backLeftServo.setPwmRange(500, 2500);
        AnalogInput backLeftEncoder = HardwareManager.get(AnalogInput.class, "back left servo encoder");
        InfiniteServo backLeftInfiniteServo = new InfiniteServo(
                backLeftServo,
                backLeftEncoder,
                new PIDFAxis(
                        new PIDFAxis.K(
                                //0.2455, 0.1564, 0.0048, new double[]{0}, 1, 1, 0.2281, 7.8312
                                0.3081, 0.7168, 0.0017, new double[]{0}, 1, 1, 0.7728, 613
                        )
                ),
                2.088683115750305
        );
        CachedMotor backLeftMotor = new CachedMotor(HardwareManager.claim(DcMotorEx.class, "back left drive"), 0);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftModule = new SwerveModule(backLeftInfiniteServo, backLeftMotor, voltageSensor, backLeftPosition);

        CachedCRServo backRightServo = new CachedCRServo(HardwareManager.claim(CRServo.class, "back right servo"), 0);
        backRightServo.setPwmRange(500, 2500);
        AnalogInput backRightEncoder = HardwareManager.get(AnalogInput.class, "back right servo encoder");
        InfiniteServo backRightInfiniteServo = new InfiniteServo(
                backRightServo,
                backRightEncoder,
                new PIDFAxis(
                        new PIDFAxis.K(
                                //0.2455, 0.1564, 0.0048, new double[]{0}, 1, 1, 0.2281, 7.8312
                                0.3081, 0.7168, 0.0017, new double[]{0}, 1, 1, 0.7728, 613
                        )
                ),
                1.494636504889689
        );
        CachedMotor backRightMotor = new CachedMotor(HardwareManager.claim(DcMotorEx.class, "back right drive"), 0);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightModule = new SwerveModule(backRightInfiniteServo, backRightMotor, voltageSensor, backRightPosition);
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
    }
}
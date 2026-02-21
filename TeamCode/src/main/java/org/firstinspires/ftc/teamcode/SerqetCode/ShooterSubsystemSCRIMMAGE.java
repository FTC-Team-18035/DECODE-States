package org.firstinspires.ftc.teamcode.SerqetCode;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;

/**
 * Shooter subsystem handling dual flywheel motors and angle servos.
 * Target velocity is applied to both motors independently with separate PIDF controllers.
 */
public class ShooterSubsystemSCRIMMAGE {
    private static final double SERVO_MIN = 0.1809;
    private static final double SERVO_MAX = 0.2306;
    private final DcMotorEx leftShooter;
    private final DcMotorEx rightShooter;
    private final Servo pitchServoLeft;
    private final Servo pitchServoRight;
    private final Servo vaultRelease;
    private final CRServo vaultFeed;
    private final CRServo vaultFeed2;
    private final PIDFController leftController;
    private final PIDFController rightController;
    private double targetVelocity;
    private double currentServoPosition;

    public ShooterSubsystemSCRIMMAGE(HardwareMap hardwareMap) {
        leftShooter = hardwareMap.get(DcMotorEx.class, "left_shooter");
        rightShooter = hardwareMap.get(DcMotorEx.class, "right_shooter");
        pitchServoLeft = hardwareMap.get(Servo.class, "shooter_horizontal");
        pitchServoRight = hardwareMap.get(Servo.class, "shooter_vertical");
        vaultRelease = hardwareMap.get(Servo.class, "vault_release");
        vaultFeed = hardwareMap.get(CRServo.class, "vault_feed");
        vaultFeed2 = hardwareMap.get(CRServo.class, "vault_feed2");
        leftController = new PIDFController(new PIDFCoefficients(0.005, 0, 0, 0.000544));
        rightController = new PIDFController(new PIDFCoefficients(0.005, 0, 0, 0.000544));
        configureMotors();
        setServoPosition(0.205);
        targetVelocity = 0.0;
    }

    //Trajectory calculation based on targetDistance
    //-----------------------------------------------------
    public static double[] Calculate(double targetDistance) {
        double distance = targetDistance + 15;
        // define values used for all calculations
        double g = 980.0; // cm/s^2 magnitude of acceleration due to gravity
        double y0 = 53.0; // cm, target final height in goal
        double m = Math.min(-1.5, (-200.0 - y0) / distance); // The SLOPE with which an artifact will enter the goal with. NOT AN ANGLE
        double a = -(y0 / Math.pow(distance, 2)) + (m / distance); // The "a" value in the parabola equation
        double b = ((2.0 * y0) / distance) - m; // The "b" value in the parabola equation
        double rawlaunchAngle = Math.atan(b); // The launch angle in radians
        //Degraded (hopefully) double launchVelocity = (1.0 / Math.cos(rawlaunchAngle)) * Math.sqrt(g / (2.0 * a));
        double launchVelocity = ((1.0 / Math.cos(rawlaunchAngle)) * Math.sqrt(g / (2.0 * a))) * 1.23787177960363;  //12.21730476; //The launch velocity in cm/s

        double launchAngle = Math.toDegrees(rawlaunchAngle) * .00392157;
        return new double[]{launchVelocity, launchAngle}; // these are the target for the shooter's flywheel and servo angle adjustments
    }

    //-----------------------------------------------------
    private void configureMotors() {
        DcMotorEx[] motors = new DcMotorEx[]{leftShooter, rightShooter};
        for (DcMotorEx motor : motors) {
            MotorConfigurationType config = motor.getMotorType().clone();
            config.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(config);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    private double clampServo(double position) {
        return Range.clip(position, SERVO_MIN, SERVO_MAX);
    }

    public void setServoPosition(double position) {
        currentServoPosition = clampServo(position);
        pitchServoLeft.setPosition(currentServoPosition);
        pitchServoRight.setPosition(currentServoPosition);
    }

    public void setVaultReleasePosition(double position) {
        vaultRelease.setPosition(clampServo(position));
    }

    public void setTarget(double velocity, double anglePosition) {

        targetVelocity = velocity;
        setServoPosition(anglePosition);
    }

    public void stop() {
        targetVelocity = 0.0;
        leftShooter.setPower(0.0);
        rightShooter.setPower(0.0);
        leftController.reset();
        rightController.reset();
        setFeedPower(0);
    }

    public void setFeedPower(double power) {
        vaultFeed.setPower(-power);
        vaultFeed2.setPower(-power);
    }

    public void setPIDFCoefficients(PIDFCoefficients coefficients) {
        // Apply identical gains to both controllers (clone to avoid shared state).
        leftController.setCoefficients(new PIDFCoefficients(coefficients.P, coefficients.I, coefficients.D, coefficients.F));
        rightController.setCoefficients(new PIDFCoefficients(coefficients.P, coefficients.I, coefficients.D, coefficients.F));
    }

    public void resetControllers() {
        leftController.reset();
        rightController.reset();
    }

    public double getLeftVelocity() {
        return leftShooter.getVelocity();
    }

    public double getRightVelocity() {
        return rightShooter.getVelocity();
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getServoPosition() {
        return currentServoPosition;
    }

    public void update() {
        leftController.setTargetPosition(targetVelocity);
        rightController.setTargetPosition(-targetVelocity);
        leftController.updateFeedForwardInput(targetVelocity);
        rightController.updateFeedForwardInput(-targetVelocity);
        leftController.updatePosition(leftShooter.getVelocity());
        rightController.updatePosition(rightShooter.getVelocity());
        double leftPower = Range.clip(leftController.run(), -1.0, 1.0);
        double rightPower = Range.clip(rightController.run(), -1.0, 1.0);
        leftShooter.setPower(leftPower);
        rightShooter.setPower(rightPower);
        // Keep servos locked to the last commanded, clamped position for safety
        pitchServoLeft.setPosition(currentServoPosition);
        pitchServoRight.setPosition(currentServoPosition);
    }
}
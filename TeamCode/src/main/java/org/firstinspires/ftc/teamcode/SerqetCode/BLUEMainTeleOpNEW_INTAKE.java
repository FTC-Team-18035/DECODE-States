package org.firstinspires.ftc.teamcode.SerqetCode;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.SerqetCode.TrajectorySCRIMMAGE;
import org.firstinspires.ftc.teamcode.SerqetCode.ShooterSubsystemSCRIMMAGE;

@TeleOp(name = "BLUE Main TeleOp", group = "PedroPathing")
public class BLUEMainTeleOpNEW_INTAKE extends LinearOpMode {

    /* =========================================================
       LIMELIGHT GEOMETRY CONSTANTS
       ---------------------------------------------------------
       Used to compute distance to target from vertical angle.
       Units are meters for heights, degrees for angles.
       ========================================================= */
    private static final double LIMELIGHT_HEIGHT = 0.24;
    private static final double TARGET_HEIGHT = 0.75;
    private static final double LIMELIGHT_MOUNT_ANGLE = 13.0;

    /* =========================================================
       ALIGNMENT CONTROL CONSTANTS
       ---------------------------------------------------------
       These control how aggressively and how precisely the robot
       attempts to rotate to face the AprilTag.
       ========================================================= */

    // Proportional gain converting heading error → turn power
    private static final double ALIGN_KP = -0.015;

    // Minimum turn command to overcome drivetrain friction
    private static final double ALIGN_MIN_CMD = 0.09;

    // "Good enough" heading accuracy (degrees)
    private static final double ALIGN_ACCEPTABLE_ERROR = 0.35;

    // Smallest improvement considered meaningful (degrees)
    private static final double ALIGN_MIN_IMPROVEMENT = 0.02;

    // How many loops with no improvement we allow before declaring a stall
    private static final int ALIGN_STALL_CYCLES = 8;

    // Counts consecutive loops without improvement
    private int alignStallCounter = 0;

    /* =========================================================
       SHOOTER SAFETIES & FILTERING
       ========================================================= */

    // Allowed error between target and actual flywheel velocity
    private static final double FLYWHEEL_TOLERANCE = 50;

    // If the tag disappears for too long, abort the shot
    private static final long TAG_TIMEOUT_MS = 500;

    // Exponential smoothing factor for distance calculation
    private static final double ALPHA = 0.3;

    // Telemetry values
    public double leftError;
    public double rightError;
    public double pullBackTicks = -60;

    /* =========================================================
       HARDWARE
       ========================================================= */
    private Limelight3A limelight;
    private Follower follower;
    private ShooterSubsystemSCRIMMAGE shooter;
    private DcMotorEx intake, lift;

    /* =========================================================
       SHOOTING STATE MACHINE
       ========================================================= */
    private enum ShootState {
        IDLE,        // Driver control
        ALIGNING,    // Vision-based heading refinement
        SPINNING_UP, // Flywheel + hood positioning
        FEEDING      // Launch the note
    }

    private ShootState shootState = ShootState.IDLE;

    /* =========================================================
       ALIGNMENT TRACKING
       ========================================================= */

    // Best heading error seen so far during this alignment attempt
    private double bestHeadingErrorDeg = Double.MAX_VALUE;

    /* =========================================================
       DISTANCE TRACKING
       ========================================================= */
    private Double smoothedDistanceCm = null;
    private double targetDistanceCm = 0.0;

    /* =========================================================
       TAG VISIBILITY TRACKING
       ========================================================= */
    private long lastTagSeenTimeMs = 0;
    private double heading = 0;

    public double scalar;
    public boolean pullBackStarted = false;

    @Override
    public void runOpMode() {

        /* ---------------- Hardware Initialization ---------------- */
        shooter = new ShooterSubsystemSCRIMMAGE(hardwareMap);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lift = hardwareMap.get(DcMotorEx.class, "lift");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setTargetPosition(0);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(6); // AprilTag pipeline
        limelight.start();

        /* ---------------- Drive / Localization Setup ---------------- */
        MecanumConstants mecanumConstants = new MecanumConstants()
                .leftFrontMotorName("front_left")
                .leftRearMotorName("back_left")
                .rightFrontMotorName("front_right")
                .rightRearMotorName("back_right");

        PinpointConstants pinpointConstants = new PinpointConstants()
                .hardwareMapName("pinpoint");

        follower = new FollowerBuilder(new FollowerConstants(), hardwareMap)
                .mecanumDrivetrain(mecanumConstants)
                .pinpointLocalizer(pinpointConstants)
                .build();

        waitForStart();
        follower.startTeleOpDrive();

        /* ---------------- Main Loop ---------------- */
        while (opModeIsActive()) {
            handleDrive();
            handleIntake();
            handleShootingStateMachine();
            handleLift();

            handlePullBack(pullBackTicks);


            shooter.update();
            follower.update();
            telemetry.update();
        }

        limelight.stop();
    }

    /* =========================================================
       DRIVER CONTROL
       ========================================================= */
    private void handleDrive() {
        // Disable manual driving during shooting sequence
        if (shootState != ShootState.IDLE) return;

        if (gamepad1.left_trigger > .5) {
            scalar = 1;
        } else if (gamepad1.left_trigger < .5) {
            scalar = .5;
        }

        if (gamepad1.left_trigger > .75 && gamepad1.right_trigger > .75) {
            heading = follower.getHeading();
        }
        follower.setTeleOpDrive(
                scalar * gamepad1.left_stick_y,
                scalar * gamepad1.left_stick_x,
                scalar * gamepad1.right_stick_x,
                false,
                heading
        );
    }

    /* =========================================================
       INTAKE CONTROL
       ========================================================= */
    private void handleIntake() {
        if (shootState != ShootState.IDLE) return;

        if (gamepad1.left_bumper) {
            intake.setPower(1.0);
            shooter.setTarget(-250, .205);
        } else if (gamepad1.right_bumper) {
            intake.setPower(-1.0);
        } else {
            intake.setPower(0.0);
            shooter.setTarget(0, .205);
        }
    }

    /* =========================================================
       SHOOTING STATE MACHINE
       ========================================================= */
    private void handleShootingStateMachine() {

        /* -------- Abort if trigger released -------- */
        if (!gamepad1.a && shootState != ShootState.IDLE) {
            abortShot();
            return;
        }

        /* -------- Start shooting sequence -------- */
        if (gamepad1.a && shootState == ShootState.IDLE) {
            shootState = ShootState.ALIGNING;
            bestHeadingErrorDeg = Double.MAX_VALUE;
            alignStallCounter = 0;
            smoothedDistanceCm = null;
        }

        LLResult result = limelight.getLatestResult();
        boolean tagValid = result != null && result.isValid()
                && !result.getFiducialResults().isEmpty();

        if (tagValid) {
            lastTagSeenTimeMs = System.currentTimeMillis();
        }

        /* -------- Safety: lost tag -------- */
        if (shootState != ShootState.IDLE &&
                System.currentTimeMillis() - lastTagSeenTimeMs > TAG_TIMEOUT_MS) {
            abortShot();
            return;
        }

        switch (shootState) {

            /* =====================================================
               ALIGNING
               -----------------------------------------------------
               Continues refining heading as long as improvement
               is possible. Exits only when:
                 - Error is already acceptable AND
                 - Further improvement has stalled
               ===================================================== */
            case ALIGNING: {

                // Manual override → skip alignment and keep scoring
                if (gamepad1.b) {
                    shootState = ShootState.SPINNING_UP;
                    break;
                }

                // Horizontal offset from Limelight (degrees)
                double tx = result.getTx();
                double absError = Math.abs(tx);

                /* ----- Track whether we're still improving ----- */
                if (absError < bestHeadingErrorDeg - ALIGN_MIN_IMPROVEMENT) {
                    bestHeadingErrorDeg = absError;
                    alignStallCounter = 0;
                } else {
                    alignStallCounter++;
                }

                /* ----- Compute turn command ----- */
                double turn = ALIGN_KP * (-tx);

                // Enforce minimum command if still meaningfully off
                if (Math.abs(turn) < ALIGN_MIN_CMD &&
                        absError > ALIGN_ACCEPTABLE_ERROR) {
                    turn = Math.copySign(ALIGN_MIN_CMD, turn);
                }

                follower.setTeleOpDrive(0, 0, turn, false, heading);

                /* ----- Exit condition ----- */
                if (absError <= ALIGN_ACCEPTABLE_ERROR &&
                        alignStallCounter >= ALIGN_STALL_CYCLES) {
                    shootState = ShootState.SPINNING_UP;
                }

                break;
            }

            /* =====================================================
               SPINNING UP
               ===================================================== */
            case SPINNING_UP: {
                follower.setTeleOpDrive(0, 0, 0, false, heading);

                double distanceMeters =
                        (TARGET_HEIGHT - LIMELIGHT_HEIGHT) /
                                Math.tan(Math.toRadians(result.getTy()
                                        + LIMELIGHT_MOUNT_ANGLE));

                double distanceCm = distanceMeters * 100.0;

                smoothedDistanceCm = smoothedDistanceCm == null
                        ? distanceCm
                        : ALPHA * distanceCm + (1 - ALPHA) * smoothedDistanceCm;

                targetDistanceCm = smoothedDistanceCm;

                double targetVelocity =
                        TrajectorySCRIMMAGE.CalculateVelocity(targetDistanceCm);
                double targetAngle =
                        TrajectorySCRIMMAGE.CalculateAngle(targetDistanceCm);

                shooter.setTarget(targetVelocity, targetAngle);

                leftError = Math.abs(Math.abs(
                        shooter.getLeftVelocity()) - targetVelocity);
                rightError = Math.abs(Math.abs(
                        shooter.getRightVelocity()) - targetVelocity);

                if (leftError < FLYWHEEL_TOLERANCE ||
                        rightError < FLYWHEEL_TOLERANCE) {
                    shootState = ShootState.FEEDING;
                }

                break;
            }

            /* =====================================================
               FEEDING
               ===================================================== */
            case FEEDING:
                follower.setTeleOpDrive(0, 0, 0, false, heading);
                intake.setPower(1);
                break;

            default:
                break;
        }

        telemetry.addData("Shoot State", shootState);
        telemetry.addData("Flywheel Error",
                (leftError + rightError) / 2.0);
        telemetry.addData("Best Align Error (deg)",
                bestHeadingErrorDeg);
        telemetry.addData("Distance (cm)", targetDistanceCm);
    }

    /* =========================================================
       ABORT
       ========================================================= */
    private void abortShot() {
        shootState = ShootState.IDLE;
        shooter.stop();
        intake.setPower(0);
        smoothedDistanceCm = null;
    }

    private void handlePullBack(double target) {
        if (shootState == ShootState.SPINNING_UP) return;
        if (gamepad1.xWasPressed()) {
            intake.setPower(0);
            intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            intake.setTargetPosition(0);
            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            pullBackStarted = true;
        }
        if (shootState != ShootState.SPINNING_UP && pullBackStarted) {
            if (intake.getCurrentPosition() > target) {
                intake.setPower(-1);
                shooter.setTarget(-250, .205);
            } else {
                intake.setPower(0);
                shooter.setTarget(0, .205);
                pullBackStarted = false;
            }
        }

    }

    private void handleLift() {
        if (gamepad1.dpad_up && gamepad1.left_trigger > .75 && lift.getCurrentPosition() < 3600) {    //TODO Changed it so you have to be holding the left trigger to run the lift
            lift.setPower(1);
        } else {
            lift.setPower(0);
        }
    }
}

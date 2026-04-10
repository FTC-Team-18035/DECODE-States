package org.firstinspires.ftc.teamcode.testCode; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SerqetCode.ShooterSubsystemSCRIMMAGE;
import org.firstinspires.ftc.teamcode.SerqetCode.TrajectorySCRIMMAGE;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous(name = "Blue Near 1 Spike", group = "Examples", preselectTeleOp = "BLUE TeleOp_QuickShot")
//TODO NEAR 1st SPIKE
public class BLUE_Near_1stSpike extends OpMode {

    private static final double SHOOT_SECONDS = 1.75;           // TODO: Change this if isn't enough time or too much...6 was too much
    private static final double INTAKE_DISTANCE = 2250;
    public double leftError;
    public double rightError;
    private static final double FLYWHEEL_TOLERANCE = 50;
    private static final double MAX_DRIVE_SPEED = .8; // Change this for the max speed
    private static final double MAX_INTAKE_SPEED = .6; // Change this if we need to intake slower
    private static final double DRIVE_POWER = 0.7;
    private static final double DRIVE_TIMEOUT_SECONDS = 20.0;

    // Optional start delay so you can avoid alliance partners.
    private static final double START_DELAY_SECONDS = 0; //TODO: add delay if need to wait for alliance partner to complete tasks

    // If robot drives the wrong direction, flip this between -1 and +1.
    private static final double FORWARD_SIGN = 1.0;

    /* =========================================================
       LIMELIGHT GEOMETRY CONSTANTS (matches TeleOp)
       ========================================================= */
    private static final double LIMELIGHT_HEIGHT = 0.24;
    private static final double TARGET_HEIGHT = 0.75;
    private static final double LIMELIGHT_MOUNT_ANGLE = 13.0;

    /* =========================================================
       ALIGNMENT CONTROL CONSTANTS (matches TeleOp)
        ========================================================= */
    private static final double ALIGN_KP = -0.015;
    private static final double ALIGN_MIN_CMD = 0.09;
    private static final double ALIGN_ACCEPTABLE_ERROR = 0.35;
    private static final double ALIGN_MIN_IMPROVEMENT = 0.02;
    private static final int ALIGN_STALL_CYCLES = 8;
    private static final double ALIGN_TIMEOUT_SECONDS = 3.0;

    // Blue-side horizontal offset used in BLUEMainTeleOpWORKING
    private static final double BLUE_TX_OFFSET_DEG = -2.0;

    /* =========================================================
       SHOOTER FILTERING / SAFETIES
       ========================================================= */
    private static final double ALPHA = 0.3;
    private static final double DEFAULT_DISTANCE_CM = 260.0; // fallback if tag isn't visible

    /* =========================================================
       HOLD (to keep robot stable while shooting)
       ========================================================= */
    private static final double HOLD_HEADING_KP = 1.8;
    private static final double HOLD_MAX_TURN = 0.45;
    private static final double HOLD_MIN_TURN_CMD = 0.06;
    private static final double HOLD_HEADING_DEADBAND_RAD = Math.toRadians(1.0);

        /* =========================================================
       BLACKBOARD
       ========================================================= */

    public static final String X_KEY = "hash X";
    public static final String Y_KEY = "hash Y";
    public static final String Z_KEY = "hash Z";
    public static Double X_VALUE = null;
    public static Double Y_VALUE = null;
    public static Double Z_VALUE = null;

    private Limelight3A limelight;

    private Follower follower;
    private ShooterSubsystemSCRIMMAGE shooter;

    private DcMotorEx intake;

    private int alignStallCounter = 0;
    private double bestHeadingErrorDeg = Double.MAX_VALUE;

    private Double smoothedDistanceCm = null;
    private double targetDistanceCm = DEFAULT_DISTANCE_CM;
    private boolean pullBackStarted = false;
    private boolean intaking = false;
    private boolean startFeeding = false;
    private boolean intakeReset = false;

    private BezierPoint holdPoint = null;
    private double holdHeadingRad = 0.0;
    private boolean holdInitialized = false;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private ElapsedTime intakeTimer = new ElapsedTime();

    private int pathState;

    private final Pose startPose = new Pose(56, 146, Math.toRadians(270)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(66, 110, Math.toRadians(138)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose pickup1Pose = new Pose(26, 100, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose lineup1Pose = new Pose(52, 100, Math.toRadians(180)); // Increased by 4
    private final Pose endPose = new Pose(27, 82, Math.toRadians(270));

    private Path scorePreload;
    private Path score1Path;
    private Path readyPath;
    private Path lineup1Path;
    private Path pickup1Path;
    private Path endPath;


    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading(), .8);

        readyPath = new Path(new BezierLine(scorePose, lineup1Pose));
        readyPath.setLinearHeadingInterpolation(scorePose.getHeading(), lineup1Pose.getHeading(), .8);

        lineup1Path = new Path(new BezierLine(lineup1Pose, pickup1Pose)) ;
        lineup1Path.setLinearHeadingInterpolation(lineup1Pose.getHeading(), pickup1Pose.getHeading(), .8);

        pickup1Path = new Path(new BezierLine(pickup1Pose, scorePose));
        pickup1Path.setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading(), .8);

        score1Path = new Path(new BezierLine(scorePose, endPose ));
        score1Path.setLinearHeadingInterpolation(scorePose.getHeading(), endPose.getHeading(), .8);


    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    shootForTime(SHOOT_SECONDS);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(readyPath);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    startIntaking();
                    follower.setMaxPower(MAX_INTAKE_SPEED);
                    follower.followPath(lineup1Path);
                    pathTimer.resetTimer();
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.setMaxPower(MAX_DRIVE_SPEED);
                    follower.followPath(pickup1Path);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    shootForTime(SHOOT_SECONDS);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(score1Path);
                    setPathState(-1);
                }
                break;
            case -1:
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    requestOpModeStop();
                }
        }
    }


    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();
        shooter.update();

        if (intaking) {
            intake();
        }

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();

    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        actionTimer = new Timer();
        actionTimer.resetTimer();
        opmodeTimer.resetTimer();


        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(MAX_DRIVE_SPEED);
        shooter = new ShooterSubsystemSCRIMMAGE(hardwareMap);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(6); // matches BLUEMainTeleOpWORKING
        limelight.start();

        buildPaths();
        follower.setStartingPose(startPose);

        telemetry.addLine("Blue Auto ready: Near-1st Spike");
        telemetry.update();

    }

    /**
     * This method is called continuously after Init while waiting for "play".
     **/
    @Override
    public void init_loop() {
    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        actionTimer.resetTimer();
        setPathState(0);
    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {

        Pose hashPose = follower.getPose();   // sets a Pose for blackboard retrieval in teleop
        blackboard.put(X_KEY, hashPose.getX());         // sets the X value
        blackboard.put(Y_KEY, hashPose.getY());         // sets the Y value
        blackboard.put(Z_KEY, hashPose.getHeading());   // sets the Z(heading) value
        telemetry.addData("Final Pose", hashPose);
        telemetry.update();
    }

    private void handleIntake(double target) {
        if (!pullBackStarted) {
            intake.setPower(0);
            intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            intake.setTargetPosition(0);
            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            pullBackStarted = true;
        }
        else if (pullBackStarted) {
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
    private void startIntaking() {
        resetIntake();
    }
    private void resetIntake(){
        intake.setPower(0);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setTargetPosition(0);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intaking = true;
    }

    private void intake() {
        double intakePosition = intake.getCurrentPosition();

        if(intakePosition >= INTAKE_DISTANCE) {
            intake.setPower(0);
            intaking = false;
        }
        else {
            intake.setPower(1);
        }
    }

    private double shootForTime(double seconds) {
        ElapsedTime timer = new ElapsedTime();

        while (opmodeTimer.getElapsedTimeSeconds() < 30 && timer.seconds() < seconds + .2) {
            if(timer.seconds() > seconds) {
                shooter.stop();
                intake.setPower(0);
                startFeeding = false;
            }
            else {

                follower.update();
                updateHold();
                updateDistanceAndShooterTarget();

                if(startFeeding) {
                    intake.setPower(1);
                }
                shooter.update();

                telemetry.addData("Shooting (s)", timer.seconds());
                telemetry.update();
            }
        }
        return timer.seconds();
    }

    private void stopShoot() {
        shooter.setTarget(0, .205);
        shooter.setFeedPower(0);
        shooter.update();
    }

    private void updateDistanceAndShooterTarget() {
        LLResult result = limelight.getLatestResult();
        boolean tagValid = result != null && result.isValid()
                && result.getFiducialResults() != null
                && !result.getFiducialResults().isEmpty();

        double distanceCm = targetDistanceCm;

        if (tagValid) {
            double distanceMeters = (TARGET_HEIGHT - LIMELIGHT_HEIGHT)
                    / Math.tan(Math.toRadians(result.getTy() + LIMELIGHT_MOUNT_ANGLE));
            distanceCm = distanceMeters * 100.0;
        }

        smoothedDistanceCm = smoothedDistanceCm == null
                ? distanceCm
                : ALPHA * distanceCm + (1 - ALPHA) * smoothedDistanceCm;

        targetDistanceCm = smoothedDistanceCm;

        double targetVelocity = TrajectorySCRIMMAGE.CalculateVelocity(targetDistanceCm);
        double targetAngle = TrajectorySCRIMMAGE.CalculateAngle(targetDistanceCm);
        shooter.setTarget(targetVelocity, targetAngle);

        leftError = Math.abs(Math.abs(
                shooter.getLeftVelocity()) - targetVelocity);
        rightError = Math.abs(Math.abs(
                shooter.getRightVelocity()) - targetVelocity);

        if (leftError < FLYWHEEL_TOLERANCE ||
                rightError < FLYWHEEL_TOLERANCE) {
            startFeeding = true;
        }
    }

    private void beginHoldFromCurrentPose() {
        if (holdInitialized) return;
        holdPoint = new BezierPoint(follower.getPose().getX(), follower.getPose().getY());
        holdHeadingRad = follower.getPose().getHeading();
        holdInitialized = true;
    }


    private void updateHold() {
        if (!holdInitialized) {
            beginHoldFromCurrentPose();
        }

        double headingError = angleWrapRad(holdHeadingRad - follower.getPose().getHeading());
        double turn = HOLD_HEADING_KP * headingError;

        if (Math.abs(headingError) < HOLD_HEADING_DEADBAND_RAD) {
            turn = 0.0;
        } else {
            if (Math.abs(turn) < HOLD_MIN_TURN_CMD) {
                turn = Math.copySign(HOLD_MIN_TURN_CMD, turn);
            }
            turn = clamp(turn, -HOLD_MAX_TURN, HOLD_MAX_TURN);
        }

        // follower.holdPoint(holdPoint, turn);
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private static double angleWrapRad(double radians) {
        while (radians > Math.PI) radians -= 2.0 * Math.PI;
        while (radians < -Math.PI) radians += 2.0 * Math.PI;
        return radians;
    }
}





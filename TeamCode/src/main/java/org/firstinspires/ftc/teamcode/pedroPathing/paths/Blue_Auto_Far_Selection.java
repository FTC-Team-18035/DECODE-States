package org.firstinspires.ftc.teamcode.pedroPathing.paths; // make sure this aligns with class location

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.telemetry.SelectableOpMode;
import com.pedropathing.util.PoseHistory;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SerqetCode.BLUEMainTeleOpNEW_INTAKE;
import org.firstinspires.ftc.teamcode.SerqetCode.TrajectorySCRIMMAGE;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import org.firstinspires.ftc.teamcode.SerqetCode.ShooterSubsystemSCRIMMAGE;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Blue Auto Selection", group = "Examples", preselectTeleOp = "BLUE Main TeleOp")
 public class Blue_Auto_Far_Selection extends SelectableOpMode {
    public static Follower follower;

    @IgnoreConfigurable
    static PoseHistory poseHistory;

    @IgnoreConfigurable
    static TelemetryManager telemetryM;

    @IgnoreConfigurable
    static ArrayList<String> changes = new ArrayList<>();

    public Blue_Auto_Far_Selection() {
        super("Select a Autonomous", s -> {
            s.folder("Far Spikes", l -> {
                l.add("1st Spike", Far_Blue_1stSpike::new);
                l.add("2nd Spike", Far_Blue_2ndSpike::new);
                l.add("3rd Spike", Far_Blue_3rdSpike::new);
            });
            s.folder("Near Spike", n -> {
                n.add("1st Spike", Near_Blue_1stSpike::new);
                n.add("2nd Spike", Near_Blue_2ndSpike::new);
                n.add("3rd Spike", Near_Blue_3rdSpike::new);
            });
        });
    }


    @Override
    public void onSelect() {
        if (follower == null) {
            follower = Constants.createFollower(hardwareMap);
            PanelsConfigurables.INSTANCE.refreshClass(this);
        } else {
            follower = Constants.createFollower(hardwareMap);
        }

        follower.setStartingPose(new Pose());

        poseHistory = follower.getPoseHistory();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        Drawings.init();
    }

    @Override
    public void onLog(List<String> lines) {}

    public static void drawOnlyCurrent() {
        try {
            Drawings.drawRobot(follower.getPose());
            Drawings.sendPacket();
        } catch (Exception e) {
            throw new RuntimeException("Drawing failed " + e);
        }
    }

    public static void draw() {
        Drawings.drawDebug(follower);
    }

    /** This creates a full stop of the robot by setting the drive motors to run at 0 power. */
    public static void stopRobot() {
        follower.startTeleopDrive(true);
        follower.setTeleOpDrive(0,0,0,true);
    }

}

//TODO PRELOAD
//@Autonomous(preselectTeleOp = "BLUE Main TeleOp")
class Far_Blue_Preload extends LinearOpMode {
    private DcMotorEx frontRight, frontLeft, backRight, backLeft;

    /* =========================================================
       TUNABLE AUTONOMOUS CONSTANTS
       ========================================================= */
    private static final double SHOOT_SECONDS = 4.5;           // TODO: Change this if isn't enough time or too much...6 was too much
    private static final double DRIVE_FORWARD_INCHES = 20.0; //TODO: Change if distance is wrong

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

    private Limelight3A limelight;
    private Follower follower;
    private ShooterSubsystemSCRIMMAGE shooter;

    private int alignStallCounter = 0;
    private double bestHeadingErrorDeg = Double.MAX_VALUE;

    private Double smoothedDistanceCm = null;
    private double targetDistanceCm = DEFAULT_DISTANCE_CM;

    private BezierPoint holdPoint = null;
    private double holdHeadingRad = 0.0;
    private boolean holdInitialized = false;

    @Override
    public void runOpMode() {
        frontRight = hardwareMap.get(DcMotorEx.class, "front_right");
        frontLeft = hardwareMap.get(DcMotorEx.class, "front_left");
        backRight = hardwareMap.get(DcMotorEx.class, "back_right");
        backLeft = hardwareMap.get(DcMotorEx.class, "back_left");

        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        shooter = new ShooterSubsystemSCRIMMAGE(hardwareMap);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(6); // matches BLUEMainTeleOpWORKING
        limelight.start();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));

        telemetry.addLine("BLUE Auto ready: align -> shoot -> drive 12\"");
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) {
            limelight.stop();
            return;
        }

        follower.startTeleOpDrive();

        runStartDelay(START_DELAY_SECONDS);

        //alignToAprilTag();
        //beginHoldFromCurrentPose();

        shootForTime(SHOOT_SECONDS);
        stopShooting();

        frontLeft.setPower(-.6);
        backLeft.setPower(-.6);
        frontRight.setPower(.6);
        backRight.setPower(.6);
        sleep(250);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);


        driveForwardInches(DRIVE_FORWARD_INCHES); //TODO: This method holds position!! Don't know why. TeleOp?
        stopDrive();

        limelight.stop();
    }

    private void runStartDelay(double seconds) {
        if (seconds <= 0.0) return;

        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < seconds) {
            double remaining = Math.max(0.0, seconds - timer.seconds());

            // Keep drivetrain stopped during delay.
            follower.setTeleOpDrive(0, 0, 0, false);
            follower.update();

            telemetry.addData("Start delay (s)", remaining);
            telemetry.update();
            idle();
        }
    }

    private void alignToAprilTag() {
        alignStallCounter = 0;
        bestHeadingErrorDeg = Double.MAX_VALUE;

        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < ALIGN_TIMEOUT_SECONDS) {
            follower.update();

            LLResult result = limelight.getLatestResult();
            boolean tagValid = result != null && result.isValid()
                    && result.getFiducialResults() != null
                    && !result.getFiducialResults().isEmpty();

            if (!tagValid) {
                follower.setTeleOpDrive(0, 0, 0, false);
                telemetry.addLine("Align: no tag yet");
                telemetry.update();
                continue;
            }

            double tx = result.getTx() + BLUE_TX_OFFSET_DEG;
            double absError = Math.abs(tx);

            if (absError < bestHeadingErrorDeg - ALIGN_MIN_IMPROVEMENT) {
                bestHeadingErrorDeg = absError;
                alignStallCounter = 0;
            } else {
                alignStallCounter++;
            }

            double turn = ALIGN_KP * (-tx);

            if (Math.abs(turn) < ALIGN_MIN_CMD && absError > ALIGN_ACCEPTABLE_ERROR) {
                turn = Math.copySign(ALIGN_MIN_CMD, turn);
            }

            follower.setTeleOpDrive(0, 0, turn, false);

            telemetry.addData("Align tx", tx);
            telemetry.addData("Align abs", absError);
            telemetry.addData("Align stall", alignStallCounter);
            telemetry.update();

            if (absError <= ALIGN_ACCEPTABLE_ERROR && alignStallCounter >= ALIGN_STALL_CYCLES) {
                follower.setTeleOpDrive(0, 0, 0, false);
                return;
            }
        }

        // timeout: stop turning
        follower.setTeleOpDrive(0, 0, 0, false);
    }

    private void shootForTime(double seconds) {
        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive() && timer.seconds() < seconds) {
            follower.update();
            updateHold();

            updateDistanceAndShooterTarget();

            shooter.setFeedPower(-1.0); // matches BLUEMainTeleOpWORKING feeding direction
            shooter.update();

            telemetry.addData("Shooting (s)", timer.seconds());
            telemetry.addData("Distance (cm)", targetDistanceCm);
            telemetry.update();
        }
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
    }

    private void stopShooting() {
        shooter.setFeedPower(0.0);
        shooter.stop();
    }

    //TODO: for some reason, this method holds position nearly perfectly, find out why.
    /*private void driveForwardInches(double inches) {
        Pose startPose = follower.getPose();
        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive() && timer.seconds() < DRIVE_TIMEOUT_SECONDS) {

            Pose currentPose = follower.getPose();
            double dx = currentPose.getX() - startPose.getX();
            double dy = currentPose.getY() - startPose.getY();
            double traveled = Math.hypot(dx, dy);

            if (traveled >= inches) {
                break;
            }

            follower.setTeleOpDrive(-.75, 0, 0, false);
            telemetry.addData("Drive traveled (in)", traveled);
            telemetry.update();
        }
    }
     */
    private void driveForwardInches(double inches) {
        Pose startPose = follower.getPose();
        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive() && timer.seconds() < DRIVE_TIMEOUT_SECONDS) {

            follower.update();

            Pose currentPose = follower.getPose();
            double dx = currentPose.getX() - startPose.getX();
            double dy = currentPose.getY() - startPose.getY();
            double traveled = Math.hypot(dx, dy);

            if (traveled >= inches) {
                frontRight.setPower(0);
                frontLeft.setPower(0);
                backRight.setPower(0);
                backLeft.setPower(0);
                break;
            }

            frontRight.setPower(.6);
            frontLeft.setPower(.6);
            backRight.setPower(.6);
            backLeft.setPower(.6);

            telemetry.addData("Drive traveled (in)", traveled);
            telemetry.update();
        }
    }

    private void stopDrive() {
        follower.setTeleOpDrive(0, 0, 0, false);
        follower.update();
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

        follower.holdPoint(holdPoint, turn);
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


//TODO 1st SPIKE MARK

//@Autonomous(preselectTeleOp = "BLUE Main TeleOp")
class Far_Blue_1stSpike extends OpMode {

    private static final double SHOOT_SECONDS = 1.75;           // TODO: Change this if isn't enough time or too much...6 was too much
    private static final double INTAKE_DISTANCE = 2275;
    private static final double DRIVE_FORWARD_INCHES = 20.0; //TODO: Change if distance is wrong

    private static final double MAX_DRIVE_SPEED = .8; // Change this for the max speed
    private static final double MAX_INTAKE_SPEED = .5; // Change this if we need to intake slower
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
    public double leftError;
    public double rightError;
    private static final double FLYWHEEL_TOLERANCE = 50;

    private int pathState;

    private final Pose startPose = new Pose(56, 8, Math.toRadians(90)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(53.6, 13/*11.1*/, Math.toRadians(108)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose pickup1Pose = new Pose(14, 44.8, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose lineup1Pose = new Pose(41, 44.8, Math.toRadians(180));


    private Path scorePreload;
    private Path score1Path;
    private Path readyPath;
    private Path lineup1Path;
    private Path pickup1Path;


    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading(), .5);

        readyPath = new Path(new BezierLine(scorePose, lineup1Pose));
        readyPath.setLinearHeadingInterpolation(scorePose.getHeading(), lineup1Pose.getHeading(), .8);

        lineup1Path = new Path(new BezierLine(lineup1Pose, pickup1Pose)) ;
        lineup1Path.setLinearHeadingInterpolation(lineup1Pose.getHeading(), pickup1Pose.getHeading(), .5);

        pickup1Path = new Path(new BezierLine(pickup1Pose, scorePose));
        pickup1Path.setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading(), .8);

        score1Path = new Path(new BezierLine(scorePose, lineup1Pose ));
        score1Path.setLinearHeadingInterpolation(scorePose.getHeading(), lineup1Pose.getHeading(), .5);


    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1_10);
                break;
            case 1_10:
                if(!follower.isBusy()) {
                    //shooter.setFeedPower(-1);
                    //shooter.setTarget(1180, 20);
                    shootForTime(SHOOT_SECONDS);
                    setPathState(1);
                }
                break;
            case 1:

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {

                    /* Score Preload */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(readyPath);
                    setPathState(1_5);
                }
                break;

            case 1_5:
                if (!follower.isBusy()) {
                    startIntaking();
                    follower.setMaxPower(MAX_INTAKE_SPEED);
                    follower.followPath(lineup1Path);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {

                    follower.setMaxPower(MAX_DRIVE_SPEED);
                    follower.followPath(pickup1Path);
                    //actionTimer.resetTimer();
                    handleIntake(-60);
                    setPathState(2_5);
                }
                break;
            case 2_5:
                // if(actionTimer.getElapsedTimeSeconds() > 1) {
                //     shooter.setFeedPower(0);
                //     intake.setPower(0);
                // }
                if(!follower.isBusy()) {
                    shootForTime(SHOOT_SECONDS);
                    setPathState(3);
                }
                break;
            case 3:
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
                //stopShoot();
            }
            else {
                //intake.setPower(1);

                follower.update();
                updateHold();

                updateDistanceAndShooterTarget();

                if(startFeeding) {
                    intake.setPower(1);
                }
                shooter.update();

                telemetry.addData("Shooting (s)", timer.seconds());
                //telemetry.addData("Distance (cm)", targetDistanceCm);
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


//TODO 2nd SPIKE MARK
//@Autonomous(preselectTeleOp = "BLUE Main TeleOp")
class Far_Blue_2ndSpike extends OpMode {

    private static final double SHOOT_SECONDS = 1.75;           // TODO: Change this if isn't enough time or too much...6 was too much
    private static final double INTAKE_DISTANCE = 2275;
    private static final double DRIVE_FORWARD_INCHES = 20.0; //TODO: Change if distance is wrong
    public double leftError;
    public double rightError;
    private static final double FLYWHEEL_TOLERANCE = 50;

    private static final double MAX_DRIVE_SPEED = .8; // Change this for the max speed
    private static final double MAX_INTAKE_SPEED = .5; // Change this if we need to intake slower
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

    private Limelight3A limelight;

    private Follower follower;
    private ShooterSubsystemSCRIMMAGE shooter;

    private DcMotorEx intake;

    private int alignStallCounter = 0;
    private double bestHeadingErrorDeg = Double.MAX_VALUE;

    private Double smoothedDistanceCm = null;
    private double targetDistanceCm = DEFAULT_DISTANCE_CM;

    private BezierPoint holdPoint = null;
    private double holdHeadingRad = 0.0;
    private boolean holdInitialized = false;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private boolean pullBackStarted = false;
    private boolean intaking = false;
    private boolean startFeeding = false;

    private int pathState;

    private final Pose startPose = new Pose(56, 8, Math.toRadians(90)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(53.6, 13/*11.1*/, Math.toRadians(108)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose score2Pose = new Pose(60, 90, Math.toRadians(135));
    private final Pose pickup1Pose = new Pose(14, 44.8, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2Pose = new Pose(14,69, Math.toRadians(180));
    private final Pose lineup1Pose = new Pose(41, 44.8, Math.toRadians(180));
    private final Pose lineup2Pose = new Pose(41.9,69,Math.toRadians(180));


    private Path scorePreload;
    private Path score1Path;
    private Path score2Path;
    private Path readyPath;
    private Path lineup1Path;
    private Path lineup2Path;
    private Path pickup1Path;
    private Path pickup2Path;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading(), .5);

        readyPath = new Path(new BezierLine(scorePose, lineup1Pose));
        readyPath.setLinearHeadingInterpolation(scorePose.getHeading(), lineup1Pose.getHeading(), .8);

        lineup1Path = new Path(new BezierLine(lineup1Pose, pickup1Pose)) ;
        lineup1Path.setLinearHeadingInterpolation(lineup1Pose.getHeading(), pickup1Pose.getHeading(), .5);

        pickup1Path = new Path(new BezierLine(pickup1Pose, scorePose));
        pickup1Path.setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading(), .8);

        score1Path = new Path(new BezierLine(scorePose, lineup2Pose ));
        score1Path.setLinearHeadingInterpolation(scorePose.getHeading(), lineup2Pose.getHeading(), .5);

        lineup2Path = new Path(new BezierLine(lineup2Pose, pickup2Pose));
        lineup2Path.setLinearHeadingInterpolation(lineup2Pose.getHeading(), pickup2Pose.getHeading(), .8);

        pickup2Path = new Path(new BezierLine(pickup2Pose, score2Pose));
        pickup2Path.setLinearHeadingInterpolation(pickup2Pose.getHeading(), score2Pose.getHeading(), .8);

        score2Path = new Path(new BezierLine(score2Pose, lineup2Pose));
        score2Path.setLinearHeadingInterpolation(score2Pose.getHeading(), lineup2Pose.getHeading(), .8);



    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1_10);
                break;
            case 1_10:
                if(!follower.isBusy()) {
                    //shooter.setFeedPower(-1);
                    //shooter.setTarget(1180, 20);
                    shootForTime(SHOOT_SECONDS);
                    setPathState(1);
                }
                break;
            case 1:

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {

                    /* Score Preload */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(readyPath);

                    setPathState(1_5);
                }
                break;

            case 1_5:
                if (!follower.isBusy()) {
                    startIntaking();
                    follower.setMaxPower(MAX_INTAKE_SPEED);
                    follower.followPath(lineup1Path);
                    pathTimer.resetTimer();
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {

                    follower.setMaxPower(MAX_DRIVE_SPEED);
                    follower.followPath(pickup1Path);
                    //actionTimer.resetTimer();
                    setPathState(2_5);
                }
                break;
            case 2_5:
                // if(actionTimer.getElapsedTimeSeconds() > 1) {
                //     shooter.setFeedPower(0);
                //     intake.setPower(0);
                // }
                if(!follower.isBusy()) {
                    shootForTime(SHOOT_SECONDS);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {

                    follower.followPath(score1Path);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy())
                {

                    //if(shootForTime(SHOOT_SECONDS) >= SHOOT_SECONDS) {
                    startIntaking();
                    follower.setMaxPower(MAX_INTAKE_SPEED);
                    follower.followPath(lineup2Path);
                    pathTimer.resetTimer();
                    setPathState(5);
                }
                //}
                break;
            case 5:

                if (!follower.isBusy())
                {
                    follower.setMaxPower(MAX_DRIVE_SPEED);
                    // actionTimer.resetTimer();
                    //handleIntake(-60);
                    follower.followPath(pickup2Path);
                    setPathState(5_5);
                }
                break;
            case 5_5:
                // if(actionTimer.getElapsedTimeSeconds() > 1) {
                //   shooter.setFeedPower(0);
                // intake.setPower(0);
                //}
                if(!follower.isBusy()) {
                    shootForTime(SHOOT_SECONDS);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy())
                {

                    //if(shootForTime(SHOOT_SECONDS) >= SHOOT_SECONDS) {
                    follower.followPath(score2Path);
                    setPathState(-1);
                }

                break;
            case -1:
                if (!follower.isBusy()) {
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
    }

    private void handleIntake(double target) {
        if (!pullBackStarted) {
            intake.setPower(0);
            intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            intake.setTargetPosition(0);
            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            pullBackStarted = true;
        }
        else{
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
                //stopShoot();
            }
            else {
                //intake.setPower(1);

                follower.update();
                updateHold();

                updateDistanceAndShooterTarget();

                if(startFeeding) {
                    intake.setPower(1);
                }
                shooter.update();

                telemetry.addData("Shooting (s)", timer.seconds());
                //telemetry.addData("Distance (cm)", targetDistanceCm);
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


//TODO 3rd SPIKE MARK

//@Autonomous(preselectTeleOp = "BLUE Main TeleOp")
class Far_Blue_3rdSpike extends OpMode {

    private static final double SHOOT_SECONDS = 1.75;           // TODO: Change this if isn't enough time or too much...6 was too much
    private static final double INTAKE_DISTANCE = 2300;
    private static final double DRIVE_FORWARD_INCHES = 20.0; //TODO: Change if distance is wrong
    public double leftError;
    public double rightError;
    private static final double FLYWHEEL_TOLERANCE = 50;

    private static final double MAX_DRIVE_SPEED = .8; // Change this for the max speed
    private static final double MAX_INTAKE_SPEED = .5; // Change this if we need to intake slower
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

    private Limelight3A limelight;

    private Follower follower;
    private ShooterSubsystemSCRIMMAGE shooter;

    private DcMotorEx intake;

    private int alignStallCounter = 0;
    private double bestHeadingErrorDeg = Double.MAX_VALUE;

    private Double smoothedDistanceCm = null;
    private double targetDistanceCm = DEFAULT_DISTANCE_CM;

    private BezierPoint holdPoint = null;
    private double holdHeadingRad = 0.0;
    private boolean holdInitialized = false;
    private boolean pullBackStarted = false;
    private boolean intaking = false;
    private boolean startFeeding = false;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    private final Pose startPose = new Pose(56, 8, Math.toRadians(90)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(53.6, 13/*11.1*/, Math.toRadians(108)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose score2Pose = new Pose(60, 90, Math.toRadians(135));
    private final Pose score3Pose = new Pose(60, 106, Math.toRadians(143));
    private final Pose pickup1Pose = new Pose(14, 44.8, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2Pose = new Pose(14, 69, Math.toRadians(180));
    private final Pose pickup3Pose = new Pose(16, 93.5, Math.toRadians(180));
    private final Pose lineup1Pose = new Pose(41, 44.8, Math.toRadians(180));
    private final Pose lineup2Pose = new Pose(41.9, 69, Math.toRadians(180));
    private final Pose lineup3Pose = new Pose(41, 93.5, Math.toRadians(180));
    private final Pose empty = new Pose(16.2, 69.8, Math.toRadians(180));
    private final Pose endPose = new Pose(53.6, 20, Math.toRadians(0));


    private Path scorePreload;
    private Path score1Path;
    private Path score2Path;
    private Path score3Path;
    private Path readyPath;
    private Path lineup1Path;
    private Path lineup2Path;
    private Path lineup3Path;
    private Path pickup1Path;
    private Path pickup2Path;
    private Path pickup3Path;
    private Path emptyPath;
    private Path endPath;
    private Path path8;
    private PathChain grabPickup1;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading(), .8);

        readyPath = new Path(new BezierLine(scorePose, lineup1Pose));
        readyPath.setLinearHeadingInterpolation(scorePose.getHeading(), lineup1Pose.getHeading(), 1);

        lineup1Path = new Path(new BezierLine(lineup1Pose, pickup1Pose));
        lineup1Path.setLinearHeadingInterpolation(lineup1Pose.getHeading(), pickup1Pose.getHeading(), .8);

        pickup1Path = new Path(new BezierLine(pickup1Pose, scorePose));
        pickup1Path.setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading(), .8);

        score1Path = new Path(new BezierLine(scorePose, lineup2Pose));
        score1Path.setLinearHeadingInterpolation(scorePose.getHeading(), lineup2Pose.getHeading(), 1);

        lineup2Path = new Path(new BezierLine(lineup2Pose, pickup2Pose));
        lineup2Path.setLinearHeadingInterpolation(lineup2Pose.getHeading(), pickup2Pose.getHeading(), .8);

        pickup2Path = new Path(new BezierLine(pickup2Pose, score2Pose));
        pickup2Path.setLinearHeadingInterpolation(pickup2Pose.getHeading(), score2Pose.getHeading(), .8);

        score2Path = new Path(new BezierLine(score2Pose, empty));
        score2Path.setLinearHeadingInterpolation(score2Pose.getHeading(), empty.getHeading(), .8);

        emptyPath = new Path(new BezierLine(empty, endPose));
        emptyPath.setLinearHeadingInterpolation(empty.getHeading(), endPose.getHeading(), .8);

        endPath = new Path(new BezierLine(scorePose, endPose));
        endPath.setLinearHeadingInterpolation(scorePose.getHeading(), endPose.getHeading(), .8);

        lineup3Path = new Path(new BezierLine(score2Pose, lineup3Pose));
        lineup3Path.setLinearHeadingInterpolation(score2Pose.getHeading(), lineup3Pose.getHeading(), 1);

        pickup3Path = new Path(new BezierLine(lineup3Pose, pickup3Pose));
        pickup3Path.setLinearHeadingInterpolation(lineup3Pose.getHeading(), pickup3Pose.getHeading(), .8);

        score3Path = new Path(new BezierLine(pickup3Pose, score3Pose));
        score3Path.setLinearHeadingInterpolation(pickup3Pose.getHeading(), score3Pose.getHeading());
        /*path8 = new Path(new BezierLine(pose8, pose8));
        path8.setLinearHeadingInterpolation(pose8.getHeading(), pose8.getHeading());


    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
       /* grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();*/

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1_10);
                break;
            case 1_10:
                if (!follower.isBusy()) {
                    //shooter.setFeedPower(-1);
                    //shooter.setTarget(1180, 20);
                    shootForTime(SHOOT_SECONDS);
                    setPathState(1);
                }
                break;
            case 1:

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {

                    /* Score Preload */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(readyPath);

                    setPathState(1_5);
                }
                break;

            case 1_5:
                if (!follower.isBusy()) {
                    startIntaking();
                    follower.setMaxPower(MAX_INTAKE_SPEED);
                    follower.followPath(lineup1Path);
                    pathTimer.resetTimer();
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    intake.setPower(0);

                    follower.setMaxPower(MAX_DRIVE_SPEED);
                    follower.followPath(pickup1Path);
                    //actionTimer.resetTimer();
                    actionTimer.resetTimer();
                   // handleIntake(-60);
                    setPathState(2_5);
                }
                break;
            case 2_5:
                // if(actionTimer.getElapsedTimeSeconds() > 1) {
                //     shooter.setFeedPower(0);
                //     intake.setPower(0);
                // }
                if (!follower.isBusy()) {
                    shootForTime(SHOOT_SECONDS);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {

                    follower.followPath(score1Path);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {

                    //if(shootForTime(SHOOT_SECONDS) >= SHOOT_SECONDS) {
                    startIntaking();
                    follower.setMaxPower(MAX_INTAKE_SPEED);
                    follower.followPath(lineup2Path);
                    pathTimer.resetTimer();
                    setPathState(5);
                }
                //}
                break;
            case 5:

                if (!follower.isBusy()) {

                    follower.setMaxPower(MAX_DRIVE_SPEED);
                    // actionTimer.resetTimer();
                    follower.followPath(pickup2Path);
                    actionTimer.resetTimer();
                   // handleIntake(-60);
                    setPathState(5_5);
                }
                break;
            case 5_5:
                // if(actionTimer.getElapsedTimeSeconds() > 1) {
                //   shooter.setFeedPower(0);
                // intake.setPower(0);
                //}
                if (!follower.isBusy()) {
                    shootForTime(SHOOT_SECONDS);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {

                    //if(shootForTime(SHOOT_SECONDS) >= SHOOT_SECONDS) {
                    follower.followPath(lineup3Path);
                    setPathState(7);
                }
                //}
                break;
            case 7:
                if (!follower.isBusy()) {
                    startIntaking();
                    follower.setMaxPower(MAX_INTAKE_SPEED);
                    follower.followPath(pickup3Path);
                    pathTimer.resetTimer();
                    setPathState(8);
                }
                break;

            case 8:
                if(!follower.isBusy()) {
                    follower.setMaxPower(MAX_DRIVE_SPEED);
                    follower.followPath(score3Path);
                    setPathState(9);
                }
                break;
            case 9:
                if(!follower.isBusy()) {
                    shootForTime(SHOOT_SECONDS);
                    setPathState(-1);
                }
                break;
            case -1:
                if (!follower.isBusy()) {
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
        shooter.update();
        autonomousPathUpdate();

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
                //stopShoot();
            }
            else {
                //intake.setPower(1);

                follower.update();
                updateHold();

                updateDistanceAndShooterTarget();

                if(startFeeding) {
                    intake.setPower(1);
                }
                shooter.update();
                telemetry.addData("left error", leftError);
                telemetry.addData("right error", rightError);
                telemetry.addData("flywheel tolerance", FLYWHEEL_TOLERANCE);
                telemetry.addData("start feeding", startFeeding);
                telemetry.addData("Shooting (s)", timer.seconds());
                //telemetry.addData("Distance (cm)", targetDistanceCm);
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


//TODO NEAR 1st SPIKE
class Near_Blue_1stSpike extends OpMode {

    private static final double SHOOT_SECONDS = 1.75;           // TODO: Change this if isn't enough time or too much...6 was too much
    private static final double INTAKE_DISTANCE = 2275;
    private static final double DRIVE_FORWARD_INCHES = 20.0; //TODO: Change if distance is wrong
    public double leftError;
    public double rightError;
    private static final double FLYWHEEL_TOLERANCE = 50;
    private static final double MAX_DRIVE_SPEED = .8; // Change this for the max speed
    private static final double MAX_INTAKE_SPEED = .5; // Change this if we need to intake slower
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
    // Increased score y by 10
    //Increased pickup 1 x by 6
    private final Pose startPose = new Pose(56, 146, Math.toRadians(270)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(66, 110, Math.toRadians(138)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose pickup1Pose = new Pose(26, 100, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose lineup1Pose = new Pose(52, 100, Math.toRadians(180)); // Increased by 4
    private final Pose endPose = new Pose(34, 88, Math.toRadians(180));

    private Path scorePreload;
    private Path score1Path;
    private Path readyPath;
    private Path lineup1Path;
    private Path pickup1Path;
    private Path endPath;


    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading(), .5);

        readyPath = new Path(new BezierLine(scorePose, lineup1Pose));
        readyPath.setLinearHeadingInterpolation(scorePose.getHeading(), lineup1Pose.getHeading(), .8);

        lineup1Path = new Path(new BezierLine(lineup1Pose, pickup1Pose)) ;
        lineup1Path.setLinearHeadingInterpolation(lineup1Pose.getHeading(), pickup1Pose.getHeading(), .5);

        pickup1Path = new Path(new BezierLine(pickup1Pose, scorePose));
        pickup1Path.setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading(), .8);

        score1Path = new Path(new BezierLine(scorePose, endPose ));
        score1Path.setLinearHeadingInterpolation(scorePose.getHeading(), endPose.getHeading(), .5);


    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1_10);
                break;
            case 1_10:
                if(!follower.isBusy()) {
                    //shooter.setFeedPower(-1);
                    //shooter.setTarget(1180, 20);
                    shootForTime(SHOOT_SECONDS);
                    setPathState(1);
                }
                break;
            case 1:

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {

                    /* Score Preload */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(readyPath);

                    setPathState(1_5);
                }
                break;

            case 1_5:
                if (!follower.isBusy()) {
                    startIntaking();
                    follower.setMaxPower(MAX_INTAKE_SPEED);
                    follower.followPath(lineup1Path);
                    pathTimer.resetTimer();
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {

                    follower.setMaxPower(MAX_DRIVE_SPEED);
                    follower.followPath(pickup1Path);
                    //actionTimer.resetTimer();
                    setPathState(2_5);
                }
                break;
            case 2_5:
                // if(actionTimer.getElapsedTimeSeconds() > 1) {
                //     intake.setPower(0);
                //     intake.setPower(0);
                // }
                if(!follower.isBusy()) {
                    shootForTime(SHOOT_SECONDS);
                    setPathState(3);
                }
                break;
            case 3:
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
                //stopShoot();
            }
            else {
                //intake.setPower(1);

                follower.update();
                updateHold();

                updateDistanceAndShooterTarget();

                if(startFeeding) {
                    intake.setPower(1);
                }
                shooter.update();

                telemetry.addData("Shooting (s)", timer.seconds());
                //telemetry.addData("Distance (cm)", targetDistanceCm);
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


//TODO NEAR 2nd SPIKE
class Near_Blue_2ndSpike extends OpMode {

    private static final double SHOOT_SECONDS = 1.75;           // TODO: Change this if isn't enough time or too much...6 was too much
    private static final double INTAKE_DISTANCE = 2275;
    private static final double DRIVE_FORWARD_INCHES = 20.0; //TODO: Change if distance is wrong
    public double leftError;
    public double rightError;
    private static final double FLYWHEEL_TOLERANCE = 50;
    private static final double MAX_DRIVE_SPEED = .8; // Change this for the max speed
    private static final double MAX_INTAKE_SPEED = .5; // Change this if we need to intake slower
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
    // Increased score y by 10
    //Increased pickup 1 x by 6
    private final Pose startPose = new Pose(56, 146, Math.toRadians(270)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(66, 110, Math.toRadians(138)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose pickup1Pose = new Pose(26, 100, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose lineup1Pose = new Pose(52, 100, Math.toRadians(180)); // Increased by 4
    private final Pose pickup2Pose = new Pose(26,76, Math.toRadians(180));  // Increased x by 8
    private final Pose lineup2Pose = new Pose(52, 76, Math.toRadians(180)); // Increased by 4
    private final Pose endPose = new Pose(34, 88, Math.toRadians(180));

    private Path scorePreload;
    private Path score1Path;
    private Path score2Path;
    private Path readyPath;
    private Path lineup1Path;
    private Path lineup2Path;
    private Path pickup1Path;
    private Path pickup2Path;
    private Path endPath;


    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading(), .3);

        readyPath = new Path(new BezierLine(scorePose, lineup1Pose));
        readyPath.setLinearHeadingInterpolation(scorePose.getHeading(), lineup1Pose.getHeading(), .8);

        lineup1Path = new Path(new BezierLine(lineup1Pose, pickup1Pose)) ;
        lineup1Path.setLinearHeadingInterpolation(lineup1Pose.getHeading(), pickup1Pose.getHeading(), .5);

        pickup1Path = new Path(new BezierLine(pickup1Pose, scorePose));
        pickup1Path.setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading(), .8);

        score1Path = new Path(new BezierLine(scorePose, lineup2Pose ));
        score1Path.setLinearHeadingInterpolation(scorePose.getHeading(), lineup2Pose.getHeading(), .5);

        lineup2Path = new Path(new BezierLine(lineup2Pose, pickup2Pose));
        lineup2Path.setLinearHeadingInterpolation(lineup2Pose.getHeading(), pickup2Pose.getHeading(), .5);

        pickup2Path = new Path(new BezierLine(pickup2Pose, scorePose));
        pickup2Path.setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading(), .8);

        endPath = new Path(new BezierLine(scorePose, endPose));
        endPath.setLinearHeadingInterpolation(scorePose.getHeading(), endPose.getHeading(), .5);
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1_10);
                break;
            case 1_10:
                if(!follower.isBusy()) {
                    //intake.setPower(1);
                    //shooter.setTarget(1180, 20);
                    shootForTime(SHOOT_SECONDS);
                    setPathState(1);
                }
                break;
            case 1:

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {

                    /* Score Preload */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(readyPath);

                    setPathState(1_5);
                }
                break;

            case 1_5:
                if (!follower.isBusy()) {
                    startIntaking();
                    follower.setMaxPower(MAX_INTAKE_SPEED);
                    follower.followPath(lineup1Path);
                    pathTimer.resetTimer();
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {

                    follower.setMaxPower(MAX_DRIVE_SPEED);
                    follower.followPath(pickup1Path);
                    //actionTimer.resetTimer();
                    setPathState(2_5);
                }
                break;
            case 2_5:
                // if(actionTimer.getElapsedTimeSeconds() > 1) {
                //     intake.setPower(0);
                //     intake.setPower(0);
                // }
                if(!follower.isBusy()) {
                    shootForTime(SHOOT_SECONDS);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {

                    follower.followPath(score1Path);
                    setPathState(3_10);
                }
                break;
            case 3_10:
                if (!follower.isBusy()) {

                    shootForTime(SHOOT_SECONDS);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    startIntaking();
                    follower.setMaxPower(MAX_INTAKE_SPEED);
                    follower.followPath(lineup2Path);
                    pathTimer.resetTimer();
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {

                    follower.setMaxPower(MAX_DRIVE_SPEED);
                    follower.followPath(pickup2Path);
                    //actionTimer.resetTimer();
                    setPathState(5_5);
                }
                break;
            case 5_5:
                if(!follower.isBusy()) {
                    shootForTime(SHOOT_SECONDS);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {

                    follower.followPath(score2Path);
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
                //stopShoot();
            }
            else {
                //intake.setPower(1);

                follower.update();
                updateHold();

                updateDistanceAndShooterTarget();

                if(startFeeding) {
                    intake.setPower(1);
                }
                shooter.update();

                telemetry.addData("Shooting (s)", timer.seconds());
                //telemetry.addData("Distance (cm)", targetDistanceCm);
                telemetry.update();
            }
        }
        return timer.seconds();
    }

    private void stopShoot() {
        shooter.setTarget(0, .205);
        intake.setPower(0);
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


//TODO NEAR 3rd SPIKE
class Near_Blue_3rdSpike extends OpMode {

    private static final double SHOOT_SECONDS = 1.75;           // TODO: Change this if isn't enough time or too much...6 was too much
    private static final double INTAKE_DISTANCE = 2275;
    private static final double DRIVE_FORWARD_INCHES = 20.0; //TODO: Change if distance is wrong
    public double leftError;
    public double rightError;
    private static final double FLYWHEEL_TOLERANCE = 50;
    private static final double MAX_DRIVE_SPEED = .8; // Change this for the max speed
    private static final double MAX_INTAKE_SPEED = .5; // Change this if we need to intake slower
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
    private final Pose score1Pose = new Pose(66, 110, Math.toRadians(138)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    //private final Pose score2Pose = new Pose(61,13 , Math.toRadians(125));
    private final Pose pickup1Pose = new Pose(26, 100, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2Pose = new Pose(26,76, Math.toRadians(180));
    private final Pose pickup3Pose = new Pose(26, 52, Math.toRadians(180));
    private final Pose lineup1Pose = new Pose(52, 100, Math.toRadians(180));
    private final Pose lineup2Pose = new Pose(52, 76, Math.toRadians(180));
    private final Pose lineup3Pose = new Pose(52, 52, Math.toRadians(180));
    private final Pose endPose = new Pose(34, 88, Math.toRadians(180));




    private Path scorePreload;
    private Path score1Path;
    private Path score2Path;
    private Path score3Path;
    private Path readyPath;
    private Path lineup1Path;
    private Path lineup2Path;
    private Path lineup3Path;
    private Path pickup1Path;
    private Path pickup2Path;
    private Path pickup3Path;
    private Path endPath;


    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, score1Pose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), score1Pose.getHeading(), .5);

        readyPath = new Path(new BezierLine(score1Pose, lineup1Pose));
        readyPath.setLinearHeadingInterpolation(score1Pose.getHeading(), lineup1Pose.getHeading(), .8);

        lineup1Path = new Path(new BezierLine(lineup1Pose, pickup1Pose)) ;
        lineup1Path.setLinearHeadingInterpolation(lineup1Pose.getHeading(), pickup1Pose.getHeading(), .5);

        pickup1Path = new Path(new BezierLine(pickup1Pose, score1Pose));
        pickup1Path.setLinearHeadingInterpolation(pickup1Pose.getHeading(), score1Pose.getHeading(), .8);

        score1Path = new Path(new BezierLine(score1Pose, lineup2Pose ));
        score1Path.setLinearHeadingInterpolation(score1Pose.getHeading(), lineup2Pose.getHeading(), .5);

        lineup2Path = new Path(new BezierLine(lineup2Pose, pickup2Pose));
        lineup2Path.setLinearHeadingInterpolation(lineup2Pose.getHeading(), pickup2Pose.getHeading(), .5);

        pickup2Path = new Path(new BezierLine(pickup2Pose, score1Pose));
        pickup2Path.setLinearHeadingInterpolation(pickup2Pose.getHeading(), score1Pose.getHeading(), .8);

        score2Path = new Path(new BezierLine(score1Pose, lineup3Pose));
        score2Path.setLinearHeadingInterpolation(score1Pose.getHeading(), lineup1Pose.getHeading(), .5);

        lineup3Path = new Path(new BezierLine(lineup3Pose, pickup3Pose));
        lineup3Path.setLinearHeadingInterpolation(lineup3Pose.getHeading(), pickup3Pose.getHeading(), .5);

        pickup3Path = new Path(new BezierLine(pickup3Pose, score1Pose));
        pickup3Path.setLinearHeadingInterpolation(pickup3Pose.getHeading(), score1Pose.getHeading(), .8);

        score3Path = new Path(new BezierLine(score1Pose, endPose));
        score3Path.setLinearHeadingInterpolation(score1Pose.getHeading(), endPose.getHeading(), .5);

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1_10);
                break;
            case 1_10:
                if(!follower.isBusy()) {
                    //intake.setPower(1);
                    //shooter.setTarget(1180, 20);
                    shootForTime(SHOOT_SECONDS);
                    setPathState(1);
                }
                break;
            case 1:

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {

                    /* Score Preload */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(readyPath);

                    setPathState(1_5);
                }
                break;

            case 1_5:
                if (!follower.isBusy()) {
                    startIntaking();
                    follower.setMaxPower(MAX_INTAKE_SPEED);
                    follower.followPath(lineup1Path);
                    pathTimer.resetTimer();
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {

                    follower.setMaxPower(MAX_DRIVE_SPEED);
                    follower.followPath(pickup1Path);
                    //actionTimer.resetTimer();
                    setPathState(2_5);
                }
                break;
            case 2_5:
                // if(actionTimer.getElapsedTimeSeconds() > 1) {
                //     intake.setPower(0);
                //     intake.setPower(0);
                // }
                if(!follower.isBusy()) {
                    shootForTime(SHOOT_SECONDS);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {

                    follower.followPath(score1Path);
                    setPathState(3_10);
                }
                break;
            case 3_10:
                if (!follower.isBusy()) {

                    shootForTime(SHOOT_SECONDS);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    startIntaking();
                    follower.setMaxPower(MAX_INTAKE_SPEED);
                    follower.followPath(lineup2Path);
                    pathTimer.resetTimer();
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {

                    follower.setMaxPower(MAX_DRIVE_SPEED);
                    follower.followPath(pickup2Path);
                    //actionTimer.resetTimer();
                    setPathState(5_5);
                }
                break;
            case 5_5:
                if(!follower.isBusy()) {
                    shootForTime(SHOOT_SECONDS);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {

                    follower.followPath(score2Path);
                    setPathState(6_10);
                }
                break;
            case 6_10:
                if (!follower.isBusy()) {

                    shootForTime(SHOOT_SECONDS);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    startIntaking();
                    follower.setMaxPower(MAX_INTAKE_SPEED);
                    follower.followPath(lineup3Path);
                    pathTimer.resetTimer();
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    follower.setMaxPower(MAX_DRIVE_SPEED);
                    follower.followPath(pickup3Path);
                    //actionTimer.resetTimer();
                    setPathState(8_8);
                }
                break;
            case 8_8:
                if(!follower.isBusy()) {
                    shootForTime(SHOOT_SECONDS);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {

                    follower.followPath(score3Path);
                    setPathState(-1);
                }
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
                //stopShoot();
            }
            else {
                //intake.setPower(1);

                follower.update();
                updateHold();

                updateDistanceAndShooterTarget();

                if(startFeeding) {
                    intake.setPower(1);
                }
                shooter.update();

                telemetry.addData("Shooting (s)", timer.seconds());
                //telemetry.addData("Distance (cm)", targetDistanceCm);
                telemetry.update();
            }
        }
        return timer.seconds();
    }

    private void stopShoot() {
        shooter.setTarget(0, .205);
        intake.setPower(0);
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



class Drawings {
    public static final double ROBOT_RADIUS = 9; // woah
    private static final FieldManager panelsField = PanelsField.INSTANCE.getField();

    private static final Style robotLook = new Style(
            "", "#3F51B5", 0.75
    );
    private static final Style historyLook = new Style(
            "", "#4CAF50", 0.75
    );

    /**
     * This prepares Panels Field for using Pedro Offsets
     */
    public static void init() {
        panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
    }

    /**
     * This draws everything that will be used in the Follower's telemetryDebug() method. This takes
     * a Follower as an input, so an instance of the DashbaordDrawingHandler class is not needed.
     *
     * @param follower Pedro Follower instance.
     */
    public static void drawDebug(Follower follower) {
        if (follower.getCurrentPath() != null) {
            drawPath(follower.getCurrentPath(), robotLook);
            Pose closestPoint = follower.getPointFromPath(follower.getCurrentPath().getClosestPointTValue());
            drawRobot(new Pose(closestPoint.getX(), closestPoint.getY(), follower.getCurrentPath().getHeadingGoal(follower.getCurrentPath().getClosestPointTValue())), robotLook);
        }
        drawPoseHistory(follower.getPoseHistory(), historyLook);
        drawRobot(follower.getPose(), historyLook);

        sendPacket();
    }

    /**
     * This draws a robot at a specified Pose with a specified
     * look. The heading is represented as a line.
     *
     * @param pose  the Pose to draw the robot at
     * @param style the parameters used to draw the robot with
     */
    public static void drawRobot(Pose pose, Style style) {
        if (pose == null || Double.isNaN(pose.getX()) || Double.isNaN(pose.getY()) || Double.isNaN(pose.getHeading())) {
            return;
        }

        panelsField.setStyle(style);
        panelsField.moveCursor(pose.getX(), pose.getY());
        panelsField.circle(ROBOT_RADIUS);

        Vector v = pose.getHeadingAsUnitVector();
        v.setMagnitude(v.getMagnitude() * ROBOT_RADIUS);
        double x1 = pose.getX() + v.getXComponent() / 2, y1 = pose.getY() + v.getYComponent() / 2;
        double x2 = pose.getX() + v.getXComponent(), y2 = pose.getY() + v.getYComponent();

        panelsField.setStyle(style);
        panelsField.moveCursor(x1, y1);
        panelsField.line(x2, y2);
    }

    /**
     * This draws a robot at a specified Pose. The heading is represented as a line.
     *
     * @param pose the Pose to draw the robot at
     */
    public static void drawRobot(Pose pose) {
        drawRobot(pose, robotLook);
    }

    /**
     * This draws a Path with a specified look.
     *
     * @param path  the Path to draw
     * @param style the parameters used to draw the Path with
     */
    public static void drawPath(Path path, Style style) {
        double[][] points = path.getPanelsDrawingPoints();

        for (int i = 0; i < points[0].length; i++) {
            for (int j = 0; j < points.length; j++) {
                if (Double.isNaN(points[j][i])) {
                    points[j][i] = 0;
                }
            }
        }

        panelsField.setStyle(style);
        panelsField.moveCursor(points[0][0], points[0][1]);
        panelsField.line(points[1][0], points[1][1]);
    }

    /**
     * This draws all the Paths in a PathChain with a
     * specified look.
     *
     * @param pathChain the PathChain to draw
     * @param style     the parameters used to draw the PathChain with
     */
    public static void drawPath(PathChain pathChain, Style style) {
        for (int i = 0; i < pathChain.size(); i++) {
            drawPath(pathChain.getPath(i), style);
        }
    }

    /**
     * This draws the pose history of the robot.
     *
     * @param poseTracker the PoseHistory to get the pose history from
     * @param style       the parameters used to draw the pose history with
     */
    public static void drawPoseHistory(PoseHistory poseTracker, Style style) {
        panelsField.setStyle(style);

        int size = poseTracker.getXPositionsArray().length;
        for (int i = 0; i < size - 1; i++) {

            panelsField.moveCursor(poseTracker.getXPositionsArray()[i], poseTracker.getYPositionsArray()[i]);
            panelsField.line(poseTracker.getXPositionsArray()[i + 1], poseTracker.getYPositionsArray()[i + 1]);
        }
    }

    /**
     * This draws the pose history of the robot.
     *
     * @param poseTracker the PoseHistory to get the pose history from
     */
    public static void drawPoseHistory(PoseHistory poseTracker) {
        drawPoseHistory(poseTracker, historyLook);
    }

    /**
     * This tries to send the current packet to FTControl Panels.
     */
    public static void sendPacket() {
        panelsField.update();
    }
}


package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// 2/19/26 copied from prior SERQUET tuning in January

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(13.739052626) // Weight in Kilograms 13.517052626
            .forwardZeroPowerAcceleration(-37.737917217396856)
            .lateralZeroPowerAcceleration(-76.35089741287847)

            .translationalPIDFCoefficients(new PIDFCoefficients(0.07, 0, 0, 0.09))  //0.13, 0, 0, 0))
            .headingPIDFCoefficients(new PIDFCoefficients(1.4, 0, 0, .8))     //1.45, 0 ,.0006 ,0));
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.01, 0, 0, 0.6, 0.03))

            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(.09, 0,.0085, .034))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(.85, 0, .03, .004))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.004, 0, 0, 0, 0.02))

            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(true);
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, .4, 1);
    // tValue is how much of the path it needs to travel before it is considered complete
    // timeout is how much time we are giving the robot to correct itself at the end of the path
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(.5)
            .rightFrontMotorName("front_right")
            .rightRearMotorName("back_right")
            .leftRearMotorName("back_left")
            .leftFrontMotorName("front_left")

            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)

            .xVelocity(71.85215795321727)
            .yVelocity(57.83920888825666);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(3.739) // updated 12/23        //  updated 12/6     -70.70786
            .strafePodX(2.759)  // updated 12/23        // updated 12/6       -94.9706
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)    // Needs checked
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);    // Needs checked


    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}

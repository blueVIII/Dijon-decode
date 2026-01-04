package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.paths.PathConstraints;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.pedropathing.control.PIDFCoefficients;



public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(13.6)   // TODO: replace with your robot's mass in kilograms
            .headingPIDFCoefficients(new PIDFCoefficients(0.8, 0, 0, 0.025))
            .forwardZeroPowerAcceleration(-38.6574)
            .lateralZeroPowerAcceleration(-71.4462)
            .centripetalScaling(0.0005)
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.01, 0, 0.00001, 0.06, 0.02))
            .translationalPIDFCoefficients(new PIDFCoefficients(0.13, 0, 0.025, 0.045));



    public static PathConstraints pathConstraints =
            new PathConstraints(0.99, 100, 1.5, 1);


    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)   // Do not exceed 1.0

            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightBack")
            .leftRearMotorName("leftBack")
            .leftFrontMotorName("leftFront")
            .xVelocity(63.2909)
            .yVelocity(51.0542)

            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);



    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)   // ← add this line

                .build();
    }
    // -------------------------
    // 3. LOCALIZER CONSTANTS (Pinpoint)
    // -------------------------
    public static PinpointConstants localizerConstants = new PinpointConstants()
            // TODO: set these to your actual odometry offsets (in inches)
            .forwardPodY(-3.30709)                   // distance of forward pod from robot center (Y)
            .strafePodX(-6.61417)                   // distance of strafe pod from robot center (X)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")       // <-- change to your I2C device name
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);
}


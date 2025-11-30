package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants();

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    // Pinpoint localizer - configured for I2C bus 1
    // Next steps:
    // 1. Configure "pinpoint" device in Robot Configuration (I2C bus 1) ✓
    // 2. Measure and set forwardPodY and strafePodX offsets
    // 3. Test encoder directions (forward pod should increase when moving forward, strafe pod when moving left)
    // 4. Run LocalizationTest OpMode to verify tracking
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(0)  // Measure: forward/backward offset of strafe (Y) pod from tracking point (inches)
            .strafePodX(0)   // Measure: left/right offset of forward (X) pod from tracking point (inches)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")  // Matches Robot Configuration name
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)  // This connects Pinpoint to Pedro Pathing
                .pathConstraints(pathConstraints)
                .build();
    }
}

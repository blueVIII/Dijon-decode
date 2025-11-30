package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * This is the LocalizationTest OpMode. This is a simple mecanum drive TeleOp that
 * displays the robot's pose (X, Y, heading) in telemetry. You should use this to
 * check the robot's localization with Pinpoint odometry.
 * 
 * Controls:
 * - Gamepad 1 left stick Y: Forward/Backward
 * - Gamepad 1 left stick X: Strafe Left/Right  
 * - Gamepad 1 right stick X: Rotate
 * 
 * @author Team Blue VIII
 */
@TeleOp(name = "Localization Test", group = "Pedro Pathing")
public class LocalizationTest extends OpMode {
    private Follower follower;

    @Override
    public void init() {
        // Create the follower using our Constants
        follower = Constants.createFollower(hardwareMap);
        
        // Set starting pose (you can adjust this to match your starting position)
        follower.setStartingPose(new Pose(0, 0));
        
        telemetry.addData("Status", "Initialized");
        telemetry.addLine("Use gamepad 1 to drive the robot");
        telemetry.addLine("Check telemetry for X, Y, and heading values");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // Update the follower to get pose data even before start
        follower.update();
        
        Pose pose = follower.getPose();
        telemetry.addData("Status", "Waiting for start...");
        telemetry.addData("X (inches)", "%.2f", pose.getX());
        telemetry.addData("Y (inches)", "%.2f", pose.getY());
        telemetry.addData("Heading (rad)", "%.2f", pose.getHeading());
        telemetry.addData("Heading (deg)", "%.2f", Math.toDegrees(pose.getHeading()));
        telemetry.update();
    }

    @Override
    public void start() {
        // Start teleop drive mode
        follower.startTeleopDrive();
        follower.update();
        
        telemetry.addData("Status", "Running - Drive the robot!");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Drive the robot using gamepad 1
        // left stick Y: forward/backward
        // left stick X: strafe left/right
        // right stick X: rotate
        follower.setTeleOpDrive(
            -gamepad1.left_stick_y,
            -gamepad1.left_stick_x,
            -gamepad1.right_stick_x,
            true
        );
        
        // Update the follower to process localization
        follower.update();
        
        // Get current pose
        Pose pose = follower.getPose();
        
        // Display position information
        telemetry.addData("Status", "Running");
        telemetry.addLine("--- Position ---");
        telemetry.addData("X (inches)", "%.2f", pose.getX());
        telemetry.addData("Y (inches)", "%.2f", pose.getY());
        telemetry.addLine("--- Heading ---");
        telemetry.addData("Heading (rad)", "%.3f", pose.getHeading());
        telemetry.addData("Heading (deg)", "%.2f", Math.toDegrees(pose.getHeading()));
        telemetry.addData("Total Heading (rad)", "%.3f", follower.getTotalHeading());
        telemetry.update();
    }

    @Override
    public void stop() {
        // Stop the robot
        follower.startTeleopDrive(true);
        follower.setTeleOpDrive(0, 0, 0, true);
        
        telemetry.addData("Status", "Stopped");
        telemetry.update();
    }
}


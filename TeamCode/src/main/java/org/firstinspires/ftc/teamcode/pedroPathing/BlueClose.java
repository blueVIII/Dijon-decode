package org.firstinspires.ftc.teamcode.pedroPathing;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teleop.DriveTrain;
import org.firstinspires.ftc.teamcode.teleop.PIDController;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;


@Autonomous(name = "Blue Close Auto", group = "Examples")
public class BlueClose extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    // Drivetrain
    private DriveTrain driveTrain;

    private Paths paths;

    // PID Controllers for the launchers
    private PIDController leftLauncherPid = new PIDController(0.06, 0, 0);
    private PIDController rightLauncherPid = new PIDController(0.06, 0, 0);

    // Launcher target velocity
    private final double LAUNCHER_TARGET_VELOCITY = 1200; // Adjust this value
    private boolean launchersOn = false;

    // Hardware
    private DcMotor intake = null;
    private DcMotorEx rightLauncher = null;
    private DcMotorEx leftLauncher = null;

    // Servos
    private CRServo rightLaunch = null;
    private CRServo leftLaunch = null;
    private Servo intakeSelect = null;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    // AprilTag
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    // Dynamic launcher target
    private double launcherTargetVelocity = 1200; // fallback default

    // Which tag are we aiming at?
    private static final int GOAL_TAG_ID = 24; // CHANGE to your actual goal tag ID


    private final Pose startPose = new Pose(122.5, 128.5, Math.toRadians(230)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(96, 96, Math.toRadians(45)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
//    private final Pose pickup1Pose = new Pose(37, 121, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
//    private final Pose pickup2Pose = new Pose(43, 130, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
//    private final Pose pickup3Pose = new Pose(49, 135, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.

    private Path scorePreload;
//    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3;

    public void buildPaths() {
        /* Drive straight from startPose to scorePose */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(
                startPose.getHeading(),
                scorePose.getHeading()
        );

        // Everything below is not needed for now
    /*
    grabPickup1 = follower.pathBuilder()
            .addPath(new BezierLine(scorePose, pickup1Pose))
            .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
            .build();
    */
    }

    public void autonomousPathUpdate() {
        switch (pathState) {

            /* ================= PRELOAD ================= */
            case 0:
                follower.followPath(paths.Shootpreload);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    setPathState(2);
                }
                break;

            case 2:
                spinUpLaunchers();
                if (launchersAtSpeed(50)) {
                    actionTimer.resetTimer();
                    setPathState(3);
                }
                break;

            case 3:
                spinUpLaunchers();
                feedOn();
                if (actionTimer.getElapsedTimeSeconds() > 0.4) {
                    feedOff();
                    setPathState(4);
                }
                break;

            /* ================= LOAD 1 ================= */
            case 4:
                stopLaunchers();
                setIntakeMode();
                intakeOn();
                follower.followPath(paths.AlligntoLoadup1stset);
                setPathState(5);
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Loadup1stset);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    intakeOff();
                    setShootMode();
                    follower.followPath(paths.Shoot1stset);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    setPathState(8);
                }
                break;

            case 8:
                spinUpLaunchers();
                if (launchersAtSpeed(50)) {
                    actionTimer.resetTimer();
                    setPathState(9);
                }
                break;

            case 9:
                spinUpLaunchers();
                feedOn();
                if (actionTimer.getElapsedTimeSeconds() > 0.4) {
                    feedOff();
                    setPathState(10);
                }
                break;

            /* ================= LOAD 2 ================= */
            case 10:
                stopLaunchers();
                setIntakeMode();
                intakeOn();
                follower.followPath(paths.AlligntoLoadup2ndset);
                setPathState(11);
                break;

            case 11:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Loadup2ndset);
                    setPathState(12);
                }
                break;

            case 12:
                if (!follower.isBusy()) {
                    intakeOff();
                    setShootMode();
                    follower.followPath(paths.Shoot2ndSet);
                    setPathState(13);
                }
                break;

            case 13:
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    setPathState(14);
                }
                break;

            case 14:
                spinUpLaunchers();
                if (launchersAtSpeed(50)) {
                    actionTimer.resetTimer();
                    setPathState(15);
                }
                break;

            case 15:
                spinUpLaunchers();
                feedOn();
                if (actionTimer.getElapsedTimeSeconds() > 0.4) {
                    feedOff();
                    setPathState(16);
                }
                break;

            /* ================= LOAD 3 ================= */
            case 16:
                stopLaunchers();
                setIntakeMode();
                intakeOn();
                follower.followPath(paths.Alligntoloadup3rdset);
                setPathState(17);
                break;

            case 17:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Loadup3rdset);
                    setPathState(18);
                }
                break;

            case 18:
                if (!follower.isBusy()) {
                    intakeOff();
                    setShootMode();
                    follower.followPath(paths.Shoot3rdset);
                    setPathState(19);
                }
                break;

            case 19:
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    setPathState(20);
                }
                break;

            case 20:
                spinUpLaunchers();
                if (launchersAtSpeed(50)) {
                    actionTimer.resetTimer();
                    setPathState(21);
                }
                break;

            case 21:
                spinUpLaunchers();
                feedOn();
                if (actionTimer.getElapsedTimeSeconds() > 0.4) {
                    feedOff();
                    setPathState(22);
                }
                break;

            /* ================= END ================= */
            case 22:
                stopLaunchers();
                intakeOff();
                setPathState(-1);
                break;
        }
    }


    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Left Vel", leftLauncher.getVelocity());
        telemetry.addData("Right Vel", rightLauncher.getVelocity());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        follower = Constants.createFollower(hardwareMap);
        paths = new Paths(follower);
        follower.setStartingPose(startPose);

        leftLauncher = hardwareMap.get(DcMotorEx.class, "leftLauncher");
        rightLauncher = hardwareMap.get(DcMotorEx.class, "rightLauncher");

        leftLaunch = hardwareMap.get(CRServo.class, "leftLaunch");
        rightLaunch = hardwareMap.get(CRServo.class, "rightLaunch");

        leftLauncher.setDirection(DcMotor.Direction.REVERSE);

        leftLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftLauncherPid = new PIDController(0.06, 0, 0);
        rightLauncherPid = new PIDController(0.06, 0, 0);

        actionTimer = new Timer();

        intake = hardwareMap.get(DcMotor.class, "intake");
        intakeSelect = hardwareMap.get(Servo.class, "intakeSelect");

    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    private void spinUpLaunchers() {
        double leftPower = leftLauncherPid.calculate(
                LAUNCHER_TARGET_VELOCITY,
                leftLauncher.getVelocity()
        );
        double rightPower = rightLauncherPid.calculate(
                LAUNCHER_TARGET_VELOCITY,
                rightLauncher.getVelocity()
        );

        leftLauncher.setPower(leftPower);
        rightLauncher.setPower(rightPower);
    }

    private boolean launchersAtSpeed(double tolerance) {
        return Math.abs(leftLauncher.getVelocity() - LAUNCHER_TARGET_VELOCITY) < tolerance
                && Math.abs(rightLauncher.getVelocity() - LAUNCHER_TARGET_VELOCITY) < tolerance;
    }

    private void feedOn() {
        leftLaunch.setPower(1.0);
        rightLaunch.setPower(1.0);
    }

    private void feedOff() {
        leftLaunch.setPower(0.0);
        rightLaunch.setPower(0.0);
    }

    private void stopLaunchers() {
        leftLauncher.setPower(0);
        rightLauncher.setPower(0);
        leftLauncherPid.reset();
        rightLauncherPid.reset();
    }

    /* ================= INTAKE HELPERS ================= */

    private void intakeOn() {
        intake.setPower(-0.8); // same as TeleOp intake-in
    }

    private void intakeOff() {
        intake.setPower(0.0);
    }

    private void intakeReverse() {
        intake.setPower(0.8);
    }

    private void setIntakeMode() {
        intakeSelect.setPosition(0.47); // TeleOp intake position
    }

    private void setShootMode() {
        intakeSelect.setPosition(0.63); // TeleOp shoot position
    }


    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}
}

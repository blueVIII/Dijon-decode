package org.firstinspires.ftc.teamcode.pedroPathing;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Paths;
import org.firstinspires.ftc.teamcode.teleop.DriveTrain;
import org.firstinspires.ftc.teamcode.teleop.PIDController;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;


@Autonomous(name = "Red Far Auto", group = "Examples")
public class RedFar extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    // Drivetrain
    private DriveTrain driveTrain;

    // April tag initialization
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    // Camera servo
    private Servo cameraServo;

    // Optional: camera servo positions
    private static final double CAM_LEFT = 0.22;
    private static final double CAM_CENTER = 0.55;
    private static final double CAM_RIGHT = 0.8;
    private int detectedTagId = -1; // -1 = none seen

    private static final double CAM_KP = 0.01;   // tune
    private static final double CAM_DEADBAND = 1.5; // degrees


    private Paths paths;

    // PID Controllers for the launchers
    private PIDController leftLauncherPid = new PIDController(0.06, 0, 0);
    private PIDController rightLauncherPid = new PIDController(0.06, 0, 0);

    // Launcher target velocity
    private final double LAUNCHER_TARGET_VELOCITY = 1200;
    // Adjust this value
    private final double LAUNCHER_STOP_VELOCITY = 0;

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
    private enum FeedSide { LEFT, RIGHT }

    private enum PreloadMode {
        SHOT_SEQUENCE,        // existing L-R-L logic (tag 22)
        LEFT_THEN_RIGHT,      // tag 21
        RIGHT_THEN_LEFT       // tag 23
    }

    private PreloadMode preloadMode = PreloadMode.SHOT_SEQUENCE;
    private int preloadContinuousPhase = 0; // 0 = first side, 1 = second side

    private static final double LEFT_UNLOAD_TIME = 4.0;   // seconds
    private static final double RIGHT_UNLOAD_TIME = 3.0;  // seconds

    private FeedSide[] preloadSequence = {};


    private int preloadShotIndex = 0;
    private int preloadPhase = 0; // 0 = feeding, 1 = recovering
    private static final double FEED_TIME = 0.6;     // tune
    private static final double RECOVER_TIME = 0.15;  // tune
    private static final double MAX_RECOVER_TIME = 0.8; // tune
    private static final double INTAKE_ASSIST_POWER = -0.9; // tune this
    private static final double INTAKE_LEAD_TIME = 0.12; // tune (0.08–0.20)
    private static final double INTAKE_DRIVE_SCALE = 0.1; // 25% speed
    private static final double LOAD1_SERVO_SWITCH_TIME = 5.5; // tune (seconds)
    private boolean load1ServoSwitched = false;


    // Dynamic launcher target
    private double launcherTargetVelocity = 1200; // fallback default

    // Which tag are we aiming at?
    private static final int GOAL_TAG_ID = 24; // CHANGE to your actual goal tag ID


    private final Pose startPose = new Pose(122.5, 128.5, Math.toRadians(230)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(108, 108, Math.toRadians(45)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
//    private final Pose pickup1Pose = new Pose(37, 121, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
//    private final Pose pickup2Pose = new Pose(43, 130, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
//    private final Pose pickup3Pose = new Pose(49, 135, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.

    private Path scorePreload;
//    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3;

    private void configurePreloadSequence(int tagId) {
        switch (tagId) {
            case 21:
                preloadMode = PreloadMode.LEFT_THEN_RIGHT;
                break;

            case 23:
                preloadMode = PreloadMode.RIGHT_THEN_LEFT;
                break;

            case 22:
            default:
                preloadMode = PreloadMode.SHOT_SEQUENCE;
                preloadSequence = new FeedSide[] {
                        FeedSide.LEFT,
                        FeedSide.RIGHT,
                        FeedSide.LEFT
                };
                break;
        }
    }

    private void initAprilTag() {

        aprilTag = new AprilTagProcessor.Builder().build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.addProcessor(aprilTag);

        visionPortal = builder.build();
    }

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
                spinUpLaunchers(); // start spinning immediately
                follower.followPath(paths.Shootpreload);
                setPathState(1);
                break;

            case 1:
                updateAprilTag();
                telemetry.addData("INIT Tag ID", detectedTagId);
                telemetry.update();
                spinUpLaunchers();

                if (!follower.isBusy() && launchersAtSpeed(50)) {
                    // HARD RESET anything that could be lingering
                    feedAllOff();
                    intakeOff();          // ensures intake motor is NOT running on shot 1
                    setShootMode();       // ensures your diverter/selector is in shoot position

                    preloadShotIndex = 0;
                    preloadPhase = 0;
                    preloadContinuousPhase = 0;
                    actionTimer.resetTimer();
                    setPathState(2);
                }
                break;

            case 2:
                spinUpLaunchers();

                // ================= TAG 22: EXISTING SHOT-BASED LOGIC =================
                if (preloadMode == PreloadMode.SHOT_SEQUENCE) {

                    if (preloadShotIndex >= preloadSequence.length) {
                        intakeAssistOff();
                        feedAllOff();
                        setPathState(4);
                        break;
                    }

                    if (preloadPhase == 0) {
                        FeedSide side = preloadSequence[preloadShotIndex];
                        double t = actionTimer.getElapsedTimeSeconds();

                        boolean useIntakeAssist = (preloadShotIndex == 1 || preloadShotIndex == 2);

                        if (useIntakeAssist) {
                            intakeAssistOn();

                            if (t < INTAKE_LEAD_TIME) {
                                feedAllOff();
                            } else {
                                if (side == FeedSide.LEFT) feedLeftOn();
                                else feedRightOn();
                            }

                            if (t > (INTAKE_LEAD_TIME + FEED_TIME)) {
                                feedAllOff();
                                intakeAssistOff();
                                actionTimer.resetTimer();
                                preloadPhase = 1;
                            }
                        } else {
                            intakeAssistOff();

                            if (side == FeedSide.LEFT) feedLeftOn();
                            else feedRightOn();

                            if (t > FEED_TIME) {
                                feedAllOff();
                                actionTimer.resetTimer();
                                preloadPhase = 1;
                            }
                        }
                    } else {
                        intakeAssistOff();
                        feedAllOff();

                        double t = actionTimer.getElapsedTimeSeconds();

                        if ((t > RECOVER_TIME && launchersAtSpeed(80)) || (t > MAX_RECOVER_TIME)) {
                            preloadShotIndex++;
                            preloadPhase = 0;
                            actionTimer.resetTimer();
                        }
                    }
                    break;
                }

                // ================= TAG 21 / 23: CONTINUOUS UNLOAD =================
                intakeAssistOn();

                if (preloadContinuousPhase == 0) {
                    // FIRST SIDE
                    if (preloadMode == PreloadMode.LEFT_THEN_RIGHT) {
                        feedLeftOn();
                        feedRightOff();
                        if (actionTimer.getElapsedTimeSeconds() > LEFT_UNLOAD_TIME) {
                            feedAllOff();
                            actionTimer.resetTimer();
                            preloadContinuousPhase = 1;
                        }
                    } else {
                        feedRightOn();
                        feedLeftOff();
                        if (actionTimer.getElapsedTimeSeconds() > RIGHT_UNLOAD_TIME) {
                            feedAllOff();
                            actionTimer.resetTimer();
                            preloadContinuousPhase = 1;
                        }
                    }
                } else {
                    // SECOND SIDE
                    if (preloadMode == PreloadMode.LEFT_THEN_RIGHT) {
                        feedRightOn();
                        feedLeftOff();
                        if (actionTimer.getElapsedTimeSeconds() > RIGHT_UNLOAD_TIME) {
                            feedAllOff();
                            intakeAssistOff();
                            setPathState(4);
                        }
                    } else {
                        feedLeftOn();
                        feedRightOff();
                        if (actionTimer.getElapsedTimeSeconds() > LEFT_UNLOAD_TIME) {
                            feedAllOff();
                            intakeAssistOff();
                            setPathState(4);
                        }
                    }
                }
                break;

            /* ================= LOAD 1 ================= */
            case 4:
                stopLaunchers();
                follower.setMaxPower(0.7);

                // SUPER slow crawl
                intakeOn();
                follower.followPath(paths.AlligntoLoadup1stset);
                setPathState(5);
                break;

            case 5:
                stopLaunchers();
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.25);
                    setShootMode();
                    intakeOn();

                    load1ServoSwitched = false;
                    actionTimer.resetTimer();

                    follower.followPath(paths.Loadup1stset); // first ball
                    setPathState(6);
                }
                break;

            case 6:
                stopLaunchers();
                if (!follower.isBusy()) {
                    follower.followPath(paths.Loadup1stsetBackup); //  backup
                    setPathState(7);
                }
                break;

            case 7:
                stopLaunchers();
                // mid-path servo switch
                if (!load1ServoSwitched &&
                        actionTimer.getElapsedTimeSeconds() > LOAD1_SERVO_SWITCH_TIME) {
                    setIntakeMode();
                    load1ServoSwitched = true;
                }

                if (!follower.isBusy()) {
                    setIntakeMode();
                    intakeOn();
                    follower.followPath(paths.Loadup1stset3rdBall); // get 3rd ball
                    setPathState(8);
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    intakeOff();
                    follower.setMaxPower(1.0);
                    follower.followPath(paths.Shoot1stset);
                    setPathState(9);
                }
                break;

            case 9:
                if (!follower.isBusy()){
                    spinUpLaunchers();
                    if (!follower.isBusy() && launchersAtSpeed(50)) {
                        // HARD RESET anything that could be lingering
                        feedAllOff();
                        intakeOff();          // ensures intake motor is NOT running on shot 1
                        setShootMode();       // ensures your diverter/selector is in shoot position

                        preloadShotIndex = 0;
                        preloadPhase = 0;
                        actionTimer.resetTimer();
                        setPathState(11);
                    }
                }
                break;

            case 11:
                spinUpLaunchers();

                // done with all 3 shots
                if (preloadShotIndex >= preloadSequence.length) {
                    intakeAssistOff();
                    feedAllOff();
                    setPathState(12);
                    break;
                }

                // Phase 0 = feeding (with optional intake lead-in)
                if (preloadPhase == 0) {
                    FeedSide side = preloadSequence[preloadShotIndex];
                    double t = actionTimer.getElapsedTimeSeconds();

                    boolean useIntakeAssist = (preloadShotIndex == 1 || preloadShotIndex == 2);

                    // --- Shot 2 & 3: intake assist FIRST, then feeder ---
                    if (useIntakeAssist) {
                        intakeAssistOn();

                        // lead-in time: feeder OFF
                        if (t < INTAKE_LEAD_TIME) {
                            feedAllOff();
                        } else {
                            // after lead-in: feeder ON
                            if (side == FeedSide.LEFT) feedLeftOn();
                            else feedRightOn();
                        }

                        // stop after total time (lead + feed)
                        if (t > (INTAKE_LEAD_TIME + FEED_TIME)) {
                            feedAllOff();
                            intakeAssistOff();
                            actionTimer.resetTimer();
                            preloadPhase = 1; // go recover
                        }

                        // --- Shot 1: feeder immediately, no intake assist ---
                    } else {
                        intakeAssistOff();

                        if (side == FeedSide.LEFT) feedLeftOn();
                        else feedRightOn();

                        if (t > FEED_TIME) {
                            feedAllOff();
                            actionTimer.resetTimer();
                            preloadPhase = 1; // go recover
                        }
                    }
                }

                // Phase 1 = recover (THIS WAS MISSING)
                else {
                    intakeAssistOff();
                    feedAllOff();

                    double t = actionTimer.getElapsedTimeSeconds();

                    // Wait a little + let flywheel climb back, but don't deadlock forever
                    if ((t > RECOVER_TIME && launchersAtSpeed(80)) || (t > MAX_RECOVER_TIME)) {
                        preloadShotIndex++;
                        preloadPhase = 0;
                        actionTimer.resetTimer();
                    }
                }
                break;

            /* ================= LOAD 2 ================= */
            case 12:
                stopLaunchers();
                setIntakeMode();
                intakeOn();
                follower.followPath(paths.AlligntoLoadup2ndset);
                setPathState(13);
                break;

            case 13:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Loadup2ndset);
                    setPathState(14);
                }
                break;

            case 14:
                if (!follower.isBusy()) {
                    intakeOff();
                    setShootMode();
                    follower.followPath(paths.Shoot2ndSet);
                    setPathState(15);
                }
                break;

            case 15:
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    setPathState(16);
                }
                break;

            case 16:
                spinUpLaunchers();
                if (launchersAtSpeed(50)) {
                    actionTimer.resetTimer();
                    setPathState(17);
                }
                break;

            case 17:
                spinUpLaunchers();
                feedOn();
                if (actionTimer.getElapsedTimeSeconds() > 0.4) {
                    feedOff();
                    setPathState(18);
                }
                break;

            /* ================= LOAD 3 ================= */
            case 18:
                stopLaunchers();
                setIntakeMode();
                intakeOn();
                follower.followPath(paths.Alligntoloadup3rdset);
                setPathState(19);
                break;

            case 19:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Loadup3rdset);
                    setPathState(20);
                }
                break;

            case 20:
                if (!follower.isBusy()) {
                    intakeOff();
                    setShootMode();
                    follower.followPath(paths.Shoot3rdset);
                    setPathState(21);
                }
                break;

            case 21:
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    setPathState(22);
                }
                break;

            case 22:
                spinUpLaunchers();
                if (launchersAtSpeed(50)) {
                    actionTimer.resetTimer();
                    setPathState(23);
                }
                break;

            case 23:
                spinUpLaunchers();
                feedOn();
                if (actionTimer.getElapsedTimeSeconds() > 0.4) {
                    feedOff();
                    setPathState(24);
                }
                break;

            /* ================= END ================= */
            case 24:
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

        cameraServo = hardwareMap.get(Servo.class, "cameraServo");
        cameraServo.setPosition(CAM_LEFT);

        initAprilTag();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {
    }


    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        configurePreloadSequence(detectedTagId);

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

    private void updateAprilTag() {
        List<AprilTagDetection> detections = aprilTag.getDetections();

        if (detections.isEmpty()) {
            detectedTagId = -1;
            return;
        }

        AprilTagDetection bestTag = null;

        // Prefer the goal tag if present
        for (AprilTagDetection tag : detections) {
            if (tag.id == GOAL_TAG_ID) {
                bestTag = tag;
                break;
            }
        }

        // Otherwise take the first tag
        if (bestTag == null) {
            bestTag = detections.get(0);
        }

        // === STORE THE ID ===
        detectedTagId = bestTag.id;

        // === OPTIONAL: camera servo aiming ===
        double error = -bestTag.ftcPose.yaw;

        if (Math.abs(error) < CAM_DEADBAND) {
            error = 0;
        }

        double targetPos = CAM_CENTER + error * CAM_KP;
        targetPos = Math.max(0.0, Math.min(1.0, targetPos));

        telemetry.addData("Detected Tag ID", detectedTagId);
        telemetry.addData("Tag Yaw", bestTag.ftcPose.yaw);
    }



    private void stopLaunchers() {
//        double leftPower = leftLauncherPid.calculate(
//                LAUNCHER_STOP_VELOCITY,
//                leftLauncher.getVelocity()
//        );
//        double rightPower = rightLauncherPid.calculate(
//                LAUNCHER_STOP_VELOCITY,
//                rightLauncher.getVelocity()
//        );
//
//        while (!launchersAtZero(30)) {
//            leftLauncher.setPower(leftPower);
//            rightLauncher.setPower(rightPower);
//        }
//        return;
        leftLauncher.setPower(-0.1);
        rightLauncher.setPower(-0.1);
    }
    private boolean launchersAtSpeed(double tolerance) {
        return Math.abs(leftLauncher.getVelocity() - LAUNCHER_TARGET_VELOCITY) < tolerance
                && Math.abs(rightLauncher.getVelocity() - LAUNCHER_TARGET_VELOCITY) < tolerance;
    }

    private boolean launchersAtZero(double tolerance) {
        return Math.abs(leftLauncher.getVelocity()) < tolerance
                && Math.abs(rightLauncher.getVelocity()) < tolerance;
    }

    private void feedOn() {
        feedLeftOn();
        feedRightOn();
    }

    private void feedOff() {
        feedAllOff();
    }
    private void feedLeftOn()  { leftLaunch.setPower(-1.0); }
    private void feedRightOn() { rightLaunch.setPower(1.0); }

    private void feedLeftOff()  { leftLaunch.setPower(0.0); }
    private void feedRightOff() { rightLaunch.setPower(0.0); }

    private void feedAllOff() {
        feedLeftOff();
        feedRightOff();
    }


    /* ================= INTAKE HELPERS ================= */

    private void intakeOn() {
        intake.setPower(-1.0); // same as TeleOp intake-in
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

    private void intakeAssistOn() {
        intake.setPower(INTAKE_ASSIST_POWER);
    }

    private void intakeAssistOff() {
        intake.setPower(0.0);
    }
    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

}

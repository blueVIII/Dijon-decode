package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.PtzControl;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Paths;
import org.firstinspires.ftc.teamcode.teleop.DriveTrain;
import org.firstinspires.ftc.teamcode.teleop.PIDController;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.List;

@Autonomous(name = "Red Close Auto", group = "Examples")
public class RedClose extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    // Drivetrain
    private DriveTrain driveTrain;

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
    private int detectedTagId = -1;

    private final FeedSide[] preloadSequence = {
            FeedSide.LEFT,   // left chamber ball #1
            FeedSide.RIGHT,  // right chamber only ball
            FeedSide.LEFT    // left chamber ball #2
    };

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

    // AprilTag
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    // Dynamic launcher target (kept, but NOT changing speed unless you choose to later)
    private double launcherTargetVelocity = 1200; // fallback default

    // Which tag are we aiming at?
    private static final int GOAL_TAG_ID = 24; // CHANGE to your actual goal tag ID

    // ===== Tag lock + tag-based shooting routine (ONLY affects shooting) =====
    private static final int DEFAULT_TAG = 22;

    private int lastSeenTagId = -1;
    private int lockedTagId = -1;

    // Routine state
    // 0 spinup gate, 1..3 phases, 99 done
    private int routineState = 99;
    private double routineStartTime = 0.0;

    // Spinup gating (copied concept from Far)
    private static final double MIN_SPINUP_TIME = 0.10;
    private static final double AT_SPEED_TOL = 50;           // same tolerance style as your original
    private static final double AT_SPEED_STABLE_TIME = 0.05;
    private double atSpeedSince = -1;

    // Tag routines (same as Far timings)
    private static final double TAG23_LEFT_TIME = 6.0;
    private static final double TAG23_RIGHT_TIME = 5.0;

    private static final double TAG21_RIGHT_WITH_INTAKE_TIME = 4.0;
    private static final double TAG21_LEFT_WITH_INTAKE_TIME = 6.0;

    private static final double TAG22_LEFT_ONLY_TIME = 1.5;
    private static final double TAG22_RIGHT_WITH_INTAKE_TIME = 2.0;
    private static final double TAG22_LEFT_WITH_INTAKE_TIME = 2.0;

    private final Pose startPose = new Pose(115.5, 128.5, Math.toRadians(90)); // Start Pose
    private final Pose scorePose = new Pose(108, 108, Math.toRadians(45));     // Score Pose

    private Path scorePreload;

    public void buildPaths() {
        /* Drive straight from startPose to scorePose */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(
                startPose.getHeading(),
                scorePose.getHeading()
        );
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
                // ONLY CHANGE: when we reach the shooting pose, start tag-based shooting routine
                spinUpLaunchers();

                if (!follower.isBusy()) {
                    startTagShootRoutine();
                    setPathState(2);
                }
                break;

            case 2:
                // ONLY CHANGE: run the tag-based routine, then continue exactly as before to LOAD1 (state 4)
                if (updateTagShootRoutine()) {
                    feedAllOff();
                    intakeOff();
                    setPathState(4);
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
                    follower.followPath(paths.Loadup1stsetBackup); // backup
                    setPathState(7);
                }
                break;

            case 7:
                stopLaunchers();
                // ✅ KEEP ORIGINAL TIMED SERVO SWITCH EXACTLY
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
                // ONLY CHANGE: when we reach this shooting pose, start tag-based shooting routine
                if (!follower.isBusy()) {
                    startTagShootRoutine();
                    setPathState(11);
                }
                break;

            case 11:
                // ONLY CHANGE: run tag routine, then continue exactly as before to LOAD2 (state 12)
                if (updateTagShootRoutine()) {
                    feedAllOff();
                    intakeOff();
                    setPathState(12);
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
        telemetry.addData("LockedTag", lockedTagId);
        telemetry.addData("ShootRoutineState", routineState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Left Vel", leftLauncher.getVelocity());
        telemetry.addData("Right Vel", rightLauncher.getVelocity());
        telemetry.update();
    }

    private void initAprilTag() {

        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")) // make sure name matches RC config
                .addProcessor(aprilTag)
                .build();

        // PTZ is OPTIONAL and camera-dependent
//        PtzControl ptzControl = visionPortal.getCameraControl(PtzControl.class);
//        if (ptzControl != null) {
//            ptzControl.setZoom(2);  // integer zoom
//        }
    }

    private void updateAprilTag() {
        if (aprilTag == null) return;
        List<AprilTagDetection> detections = aprilTag.getDetections();

        if (detections == null || detections.isEmpty()) {
            detectedTagId = -1;
            return;
        }

        detectedTagId = detections.get(0).id;
        lastSeenTagId = detectedTagId;
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

        // ✅ AprilTag init
        initAprilTag();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {
        updateAprilTag();

        if (aprilTag == null) {
            telemetry.addLine("AprilTag not initialized");
            telemetry.update();
            return;
        }

        telemetry.addData("AprilTag ID (frame)", detectedTagId);
        telemetry.addData("AprilTag Last Seen", lastSeenTagId);
        telemetry.update();
    }

    /** This method is called once at the start of the OpMode. **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();

        // ✅ lock tag ONCE at the beginning
        lockedTagId = (lastSeenTagId == -1) ? DEFAULT_TAG : lastSeenTagId;

        setPathState(0);
    }

    // ===== Shooter routine (based on lockedTagId) =====

    private void startTagShootRoutine() {
        if (lockedTagId == -1) lockedTagId = DEFAULT_TAG;

        // Clean start
        feedAllOff();
        intakeOff();
        setShootMode();

        routineState = 0;
        routineStartTime = getRuntime();
        atSpeedSince = -1;
    }

    private boolean updateTagShootRoutine() {
        // always keep flywheels spinning while routine active
        if (routineState != 99) {
            spinUpLaunchers();
        }

        double t = getRuntime() - routineStartTime;

        // default safe outputs
        feedAllOff();
        intakeOff();

        switch (routineState) {
            case 0: {
                // spinup gate (same style as Far)
                double spinT = getRuntime() - routineStartTime;
                if (spinT < MIN_SPINUP_TIME) {
                    atSpeedSince = -1;
                    return false;
                }

                if (launchersAtSpeed(AT_SPEED_TOL)) {
                    if (atSpeedSince < 0) atSpeedSince = getRuntime();
                    if ((getRuntime() - atSpeedSince) >= AT_SPEED_STABLE_TIME) {
                        routineState = 1;
                        routineStartTime = getRuntime();
                        atSpeedSince = -1;
                    }
                } else {
                    atSpeedSince = -1;
                }
                return false;
            }

            case 1:
            case 2:
            case 3:
                return runLockedTagStep(t);

            case 99:
            default:
                return true;
        }
    }

    private boolean runLockedTagStep(double t) {

        // ===== TAG 23 =====
        if (lockedTagId == 22) {
            if (routineState == 1) {
                feedLeftOn();
                intakeAssistOn();
                if (t >= TAG23_LEFT_TIME) {
                    routineState = 2;
                    routineStartTime = getRuntime();
                }
                return false;
            } else if (routineState == 2) {
                feedRightOn();
                intakeAssistOn();
                if (t >= TAG23_RIGHT_TIME) {
                    routineState = 99;
                    feedAllOff();
                    intakeOff();
                    return true;
                }
                return false;
            }
        }

        // ===== TAG 21 =====
        if (lockedTagId == 23) {
            if (routineState == 1) {
                feedRightOn();
                intakeAssistOn();
                if (t >= TAG21_RIGHT_WITH_INTAKE_TIME) {
                    routineState = 2;
                    routineStartTime = getRuntime();
                }
                return false;
            } else if (routineState == 2) {
                feedLeftOn();
                intakeAssistOn();
                if (t >= TAG21_LEFT_WITH_INTAKE_TIME) {
                    routineState = 99;
                    feedAllOff();
                    intakeOff();
                    return true;
                }
                return false;
            }
        }

        // ===== TAG 22 (default) =====
        if (routineState == 1) {
            feedLeftOn();
            // no intake
            if (t >= TAG22_LEFT_ONLY_TIME) {
                routineState = 2;
                routineStartTime = getRuntime();
            }
            return false;
        } else if (routineState == 2) {
            feedRightOn();
            intakeAssistOn();
            if (t >= TAG22_RIGHT_WITH_INTAKE_TIME) {
                routineState = 3;
                routineStartTime = getRuntime();
            }
            return false;
        } else if (routineState == 3) {
            feedLeftOn();
            intakeAssistOn();
            if (t >= TAG22_LEFT_WITH_INTAKE_TIME) {
                routineState = 99;
                feedAllOff();
                intakeOff();
                return true;
            }
            return false;
        }

        routineState = 99;
        return true;
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

    private void stopLaunchers() {
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
    public void stop() {}
}

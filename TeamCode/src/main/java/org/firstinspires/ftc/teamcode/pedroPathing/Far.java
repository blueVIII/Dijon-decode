package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.teleop.PIDController;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;



@Autonomous(name = "Far Auto", group = "Examples")
public class Far extends OpMode {

    // AprilTag
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private int detectedTagId = -1; // -1 = none

    // Launcher + feeders
    private DcMotorEx rightLauncher = null;
    private DcMotorEx leftLauncher = null;
    private CRServo rightLaunch = null;
    private CRServo leftLaunch = null;

    // Intake + diverter
    private DcMotor intake = null;
    private Servo intakeSelect = null;

    // PIDs
    private PIDController leftLauncherPid = new PIDController(0.06, 0, 0);
    private PIDController rightLauncherPid = new PIDController(0.06, 0, 0);

    // Constants (tune as needed)
    // fallback default; actual target is `launcherTargetVelocity`
    private static final double LAUNCHER_TARGET_VELOCITY_DEFAULT = 1485;
    private static final double INTAKE_ASSIST_POWER = -0.9;

    // Spin-up gating (prevents "early" feeding due to velocity noise)
    private static final double MIN_SPINUP_TIME = 0.1;        // seconds
    private static final double AT_SPEED_TOL = 4;            // velocity units tolerance
    private static final double AT_SPEED_STABLE_TIME = 0.01;  // seconds continuously at-speed
    private double atSpeedSince = -1;

    // If no tag is seen, we default to tag 22 behavior
    private static final int DEFAULT_TAG = 22;

    // Latch behavior: detectedTagId is "this frame"; lastSeenTagId persists
    private int lastSeenTagId = -1;
    private int lockedTagId = -1;

    // Launcher target can change per-tag (and mid-routine for tag 22)
    private double launcherTargetVelocity = 1478;

    // Routine state
    // 0 = spinup, 1 = phase1, 2 = phase2, 3 = phase3 (optional), 99 = done
    private int routineState = 0;
    private double stateStartTime = 0;

    // ===== Tag-specific timings (seconds) =====
    // Tag 23: left feeder+intake 6s, then right feeder+intake 5s (launchers at 1450)
    private static final double TAG23_LEFT_TIME = 6.0;
    private static final double TAG23_RIGHT_TIME = 5.0;

    // Tag 21: right feeder + intake for 4s, then left feeder + intake for 6s
    private static final double TAG21_RIGHT_WITH_INTAKE_TIME = 4.0;
    private static final double TAG21_LEFT_WITH_INTAKE_TIME = 6.0;

    // Tag 22: left feeder only 1–2s at 1400, then raise to 1450 and
    // right feeder + intake 2s, then left feeder + intake some time.
    // (Tune these as needed.)
    private static final double TAG22_LEFT_ONLY_TIME = 1.5;
    private static final double TAG22_RIGHT_WITH_INTAKE_TIME = 2.0;
    private static final double TAG22_LEFT_WITH_INTAKE_TIME = 2.0;

    private static final double TAG22_VEL_PHASE1 = 1478;
    private static final double TAG21_VEL = 1478;
    private static final double TAG23_VEL = 1478;

    @Override
    public void init() {
        // Hardware
        leftLauncher = hardwareMap.get(DcMotorEx.class, "leftLauncher");
        rightLauncher = hardwareMap.get(DcMotorEx.class, "rightLauncher");
        leftLaunch = hardwareMap.get(CRServo.class, "leftLaunch");
        rightLaunch = hardwareMap.get(CRServo.class, "rightLaunch");

        intake = hardwareMap.get(DcMotor.class, "intake");
        intakeSelect = hardwareMap.get(Servo.class, "intakeSelect");

        leftLauncher.setDirection(DcMotor.Direction.REVERSE);
        leftLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Ensure everything is off
        feedAllOff();
        intakeOff();
        setShootMode();

        // AprilTag
        initAprilTag();

        // Reset routine state
        routineState = 0;
        stateStartTime = 0;
        detectedTagId = -1;
        lastSeenTagId = -1;
        lockedTagId = -1;
        launcherTargetVelocity = LAUNCHER_TARGET_VELOCITY_DEFAULT;
        atSpeedSince = -1;
    }

    @Override
    public void init_loop() {
        updateAprilTag();
        telemetry.addData("INIT Tag ID", detectedTagId);
        telemetry.update();
    }

    @Override
    public void start() {
        // Lock in the last-seen tag from INIT. If none seen, default.
        lockedTagId = (lastSeenTagId == -1) ? DEFAULT_TAG : lastSeenTagId;

        // Set launcher target based on tag
        if (lockedTagId == 21) {
            launcherTargetVelocity = TAG21_VEL;
        } else if (lockedTagId == 23) {
            launcherTargetVelocity = TAG23_VEL;
        } else {
            // tag 22 (or default)
            launcherTargetVelocity = TAG22_VEL_PHASE1; // start at 1400
        }

        // Prep for shooting
        feedAllOff();
        intakeOff();
        setShootMode();

        routineState = 0; // spinup
        stateStartTime = getRuntime();
        atSpeedSince = -1;
    }

    @Override
    public void loop() {
        // Update tag for telemetry only (does NOT affect lockedTagId)

        telemetry.addData("INIT Last Seen Tag", lastSeenTagId);
        telemetry.addData("Locked Tag (used)", lockedTagId);
        telemetry.addData("RoutineState", routineState);
        telemetry.addData("TargetVel", launcherTargetVelocity);
        telemetry.addData("Left Vel", leftLauncher.getVelocity());
        telemetry.addData("Right Vel", rightLauncher.getVelocity());

        // Always keep flywheels commanded while routine is active
        if (routineState != 99) {
            spinUpLaunchers();
        }

        double t = getRuntime() - stateStartTime;

        switch (routineState) {
            case 0: // spinup
                feedAllOff();
                intakeOff();

                // Minimum spin time first (prevents immediate pass)
                double spinT = getRuntime() - stateStartTime;
                if (spinT < MIN_SPINUP_TIME) {
                    atSpeedSince = -1;
                    break;
                }

                // Require stable at-speed for a short window
                if (launchersAtSpeed(AT_SPEED_TOL)) {
                    if (atSpeedSince < 0) {
                        atSpeedSince = getRuntime();
                    }
                    if ((getRuntime() - atSpeedSince) >= AT_SPEED_STABLE_TIME) {
                        routineState = 1;
                        stateStartTime = getRuntime();
                        atSpeedSince = -1;
                    }
                } else {
                    atSpeedSince = -1;
                }
                break;

            case 1:
            case 2:
            case 3:
                runTagRoutineStep(t);
                break;

            case 99: // done
                stopLaunchers();
                feedAllOff();
                intakeOff();
                break;
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        // safety
        stopLaunchers();
        feedAllOff();
        intakeOff();
        if (visionPortal != null) {
            visionPortal.close();
        }
    }


    private void runTagRoutineStep(double t) {
        // Default: ensure only the intended actuators run
        feedAllOff();
        intakeOff();

        if (lockedTagId == 23) {
            // ===== TAG 23 =====
            // Launchers at 1450 (already set in start)
            // Phase1: 6s left feeder + intake
            // Phase2: 5s right feeder + intake

            if (routineState == 1) {
                feedLeftOn();
                intakeAssistOn();
                if (t >= TAG23_LEFT_TIME) {
                    routineState = 2;
                    stateStartTime = getRuntime();
                }
            } else if (routineState == 2) {
                feedRightOn();
                intakeAssistOn();
                if (t >= TAG23_RIGHT_TIME) {
                    routineState = 99;
                }
            }
            return;
        }

        if (lockedTagId == 21) {
            // ===== TAG 21 =====
            // Launchers at 1450
            // Phase1: 4s right feeder + intake
            // Phase2: 6s left feeder + intake

            if (routineState == 1) {
                // Phase 1: right feeder + intake
                feedRightOn();
                if (t >= TAG21_RIGHT_WITH_INTAKE_TIME) {
                    routineState = 2;
                    stateStartTime = getRuntime();
                }
            } else if (routineState == 2) {
                // Phase 2: left feeder + intake
                feedLeftOn();
                intakeAssistOn();
                if (t >= TAG21_LEFT_WITH_INTAKE_TIME) {
                    routineState = 99;
                }
            }
            return;
        }

        // ===== TAG 22 (default) =====
        // Phase1: target 1400, left feeder only for 1–2s (no intake)
        // Phase2: raise target to 1450, right feeder + intake for 2s
        // Phase3: left feeder + intake for some time (2s)

        if (routineState == 1) {
            launcherTargetVelocity = TAG22_VEL_PHASE1;
            feedLeftOn();
            // no intake
            if (t >= TAG22_LEFT_ONLY_TIME) {
                // bump up speed for remaining phases
                launcherTargetVelocity = LAUNCHER_TARGET_VELOCITY_DEFAULT;
                routineState = 2;
                stateStartTime = getRuntime();
            }
        } else if (routineState == 2) {
            launcherTargetVelocity = LAUNCHER_TARGET_VELOCITY_DEFAULT;
            feedRightOn();
            intakeAssistOn();
            if (t >= TAG22_RIGHT_WITH_INTAKE_TIME) {
                routineState = 3;
                stateStartTime = getRuntime();
            }
        } else if (routineState == 3) {
            launcherTargetVelocity = LAUNCHER_TARGET_VELOCITY_DEFAULT;
            feedLeftOn();
            intakeAssistOn();
            if (t >= TAG22_LEFT_WITH_INTAKE_TIME) {
                routineState = 99;
            }
        }
    }

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().build();
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
    }

    private void updateAprilTag() {
        if (aprilTag == null) return;
        List<AprilTagDetection> detections = aprilTag.getDetections();

        // If no detections this frame, keep lastSeenTagId, but show detectedTagId=-1
        if (detections == null || detections.isEmpty()) {
            detectedTagId = -1;
            return;
        }

        // Use first detection (simple)
        detectedTagId = detections.get(0).id;
        lastSeenTagId = detectedTagId;
    }

    private double clip(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private void spinUpLaunchers() {
        double leftPower = leftLauncherPid.calculate(
                launcherTargetVelocity,
                leftLauncher.getVelocity()
        );
        double rightPower = rightLauncherPid.calculate(
                launcherTargetVelocity,
                rightLauncher.getVelocity()
        );
        leftLauncher.setPower(clip(leftPower, -1.0, 1.0));
        rightLauncher.setPower(clip(rightPower, -1.0, 1.0));
    }

    private void stopLaunchers() {
        leftLauncher.setPower(0.0);
        rightLauncher.setPower(0.0);
    }

    private boolean launchersAtSpeed(double tolerance) {
        return Math.abs(leftLauncher.getVelocity() - launcherTargetVelocity) < tolerance
                && Math.abs(rightLauncher.getVelocity() - launcherTargetVelocity) < tolerance;
    }

    private void feedLeftOn()  { leftLaunch.setPower(-1.0); }
    private void feedRightOn() { rightLaunch.setPower(1.0); }

    private void feedLeftOff()  { leftLaunch.setPower(0.0); }
    private void feedRightOff() { rightLaunch.setPower(0.0); }

    private void feedAllOff() {
        feedLeftOff();
        feedRightOff();
    }

    private void intakeOff() {
        intake.setPower(0.0);
    }

    private void setShootMode() {
        intakeSelect.setPosition(0.63);
    }

    private void intakeAssistOn() {
        intake.setPower(INTAKE_ASSIST_POWER);
    }

    private void intakeAssistOff() {
        intake.setPower(0.0);
    }
}
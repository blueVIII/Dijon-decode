package org.firstinspires.ftc.teamcode.teleop;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;


@TeleOp(name="Mecanum Drive 25-26", group="Opmode")
public class TeleOpMode extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();

    // Camera + AprilTag
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    // Camera servo
    private Servo cameraServo;

    // Optional: camera servo positions
    private static final double CAM_LEFT = 0.25;
    private static final double CAM_CENTER = 0.55;
    private static final double CAM_RIGHT = 0.8;

    private static final double CAM_KP = 0.01;   // tune
    private static final double CAM_DEADBAND = 1.5; // degrees


    // Drivetrain
    private DriveTrain driveTrain;

    // PID Controllers for the launchers
    private final PIDController leftLauncherPid = new PIDController(0.06, 0, 0);
    private final PIDController rightLauncherPid = new PIDController(0.06, 0, 0);

    // Launcher target velocity
    private final double LAUNCHER_TARGET_VELOCITY = 1200; // Adjust this value
    private boolean launchersOn = false;
    private boolean autoCamera = false;


    // Hardware
    private DcMotor intake = null;
    private DcMotorEx rightLauncher = null;
    private DcMotorEx leftLauncher = null;

    // Servos
    private CRServo rightLaunch = null;
    private CRServo leftLaunch = null;
    private Servo intakeSelect = null;


    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        driveTrain = new DriveTrain(hardwareMap);

        intake = hardwareMap.get(DcMotor.class, "intake");
        rightLauncher = hardwareMap.get(DcMotorEx.class, "rightLauncher");
        leftLauncher = hardwareMap.get(DcMotorEx.class, "leftLauncher");

        rightLaunch = hardwareMap.get(CRServo.class, "rightLaunch");
        leftLaunch = hardwareMap.get(CRServo.class, "leftLaunch");
        intakeSelect = hardwareMap.get(Servo.class, "intakeSelect");

        cameraServo = hardwareMap.get(Servo.class, "cameraServo");
        cameraServo.setPosition(CAM_CENTER);

        initAprilTag();

        // Motor Settings
        leftLauncher.setDirection(DcMotor.Direction.REVERSE);

        // Use encoders for the launcher motors
        leftLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");

    }

    private void initAprilTag() {

        aprilTag = new AprilTagProcessor.Builder().build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.addProcessor(aprilTag);

        visionPortal = builder.build();
    }


    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        // Mecanum drive
        double y = -gamepad1.left_stick_y; // Reversed for consistency with original DriveTrain OpMode
        double x = gamepad1.left_stick_x * 1.5; // Strafe multiplier
        double pivot = gamepad1.right_stick_x;
        if (gamepad1.a) autoCamera = true;
        if (gamepad1.b) autoCamera = false;

        driveTrain.drive(y, x, pivot, gamepad1.left_bumper);

        // Launcher Servos
        if (gamepad2.dpad_left) {
            rightLaunch.setDirection(CRServo.Direction.FORWARD);
            rightLaunch.setPower(1.0);
        } else {
            rightLaunch.setPower(0.0);
        }

        if (gamepad2.dpad_right) {
            leftLaunch.setDirection(CRServo.Direction.REVERSE);
            leftLaunch.setPower(1.0);
        } else {
            leftLaunch.setPower(0.0);
        }

        // Camera servo control (left / center / right)
        if (gamepad1.dpad_left) {
            cameraServo.setPosition(CAM_LEFT);
        } else if (gamepad1.dpad_up) {
            cameraServo.setPosition(CAM_CENTER);
        } else if (gamepad1.dpad_right) {
            cameraServo.setPosition(CAM_RIGHT);
        }


        // Launcher Motors with PID
        if (gamepad2.left_bumper) {
            telemetry.addData("Status", "Launchers On");
            launchersOn = true;
        }
        if (gamepad2.right_bumper) {
            telemetry.addData("Status", "Launchers Off");
            launchersOn = false;
        }

        if (launchersOn) {
            double leftPower = leftLauncherPid.calculate(LAUNCHER_TARGET_VELOCITY, leftLauncher.getVelocity());
            double rightPower = rightLauncherPid.calculate(LAUNCHER_TARGET_VELOCITY, rightLauncher.getVelocity());
            leftLauncher.setPower(leftPower);
//            rightLauncher.setPower(rightPower);
        } else {
            leftLauncher.setPower(0.0);
//            rightLauncher.setPower(0.0);
            leftLauncherPid.reset();
            rightLauncherPid.reset();
        }

        // Intake
        if (gamepad2.y) {
            intake.setPower(-0.8);
        } else if (gamepad2.x){
            intake.setPower(0.8);
        } else {
            intake.setPower(0.0);
        }

        if (gamepad2.a) {
            intakeSelect.setPosition(0.47);
        }

        if (gamepad2.b) {
            intakeSelect.setPosition(0.63);
        }

        List<AprilTagDetection> detections = aprilTag.getDetections();

        if (!detections.isEmpty()) {
            AprilTagDetection tag = detections.get(0);

            // Use YAW, not bearing
            double error = -tag.ftcPose.yaw; // <-- inversion is critical

            // Deadband to prevent jitter
            if (Math.abs(error) < CAM_DEADBAND) {
                error = 0;
            }

            double targetPos = CAM_CENTER + error * CAM_KP;

            // Clamp
            targetPos = Math.max(0.0, Math.min(1.0, targetPos));

            if (autoCamera && !detections.isEmpty()) {
                // calculate servo position based on AprilTag yaw
                cameraServo.setPosition(targetPos);
            }


            telemetry.addData("Tag Yaw", tag.ftcPose.yaw);
        }

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Left Launcher Velocity", leftLauncher.getVelocity());
        telemetry.addData("Right Launcher Velocity", rightLauncher.getVelocity());
        telemetry.update();
    }

    @Override
    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

}

package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveTrain {
    private final static double CRAWL_MODE = 0.35;
    private final static double NORMAL_MODE = 0.85;
    private DcMotorEx leftFront, leftBack, rightBack, rightFront;

    public DriveTrain(HardwareMap hardwareMap) {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        // reverse any motors using DcMotor.setDirection()
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void drive(double y, double x, double pivot, boolean crawlMode) {
        double leftFrontPower = pivot + y + x;
        double leftBackPower = pivot + y - x;
        double rightFrontPower = -pivot + y - x;
        double rightBackPower = -pivot + y + x;

        double powerMode = crawlMode ? CRAWL_MODE : NORMAL_MODE;

        leftFront.setPower(leftFrontPower * powerMode);
        leftBack.setPower(leftBackPower * powerMode);
        rightFront.setPower(rightFrontPower * powerMode);
        rightBack.setPower(rightBackPower * powerMode);
    }
}

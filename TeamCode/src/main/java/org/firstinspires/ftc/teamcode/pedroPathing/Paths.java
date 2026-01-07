package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class Paths {

    // ===== Speed constants (inches per second) =====
    private static final double FAST_TRAVEL = 60;
    private static final double SLOW_INTAKE = 0.5;

    public PathChain Shootpreload;
    public PathChain AlligntoLoadup1stset;
    public PathChain Loadup1stset;
    public PathChain Loadup1stsetBackup;
    public PathChain Loadup1stset3rdBall;
    public PathChain Shoot1stset;
    public PathChain AlligntoLoadup2ndset;
    public PathChain Loadup2ndset;
    public PathChain Shoot2ndSet;
    public PathChain Alligntoloadup3rdset;
    public PathChain Loadup3rdset;
    public PathChain Shoot3rdset;

    public Paths(Follower follower) {

        // ================= PRELOAD =================
        Shootpreload = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(116.549, 130.431), new Pose(107.765, 92.471)))
                .setLinearHeadingInterpolation(Math.toRadians(216.3), Math.toRadians(45))
                .setVelocityConstraint(FAST_TRAVEL)
                .build();

        // ================= LOAD 1 =================
        AlligntoLoadup1stset = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(107.765, 92.471), new Pose(101.176, 80.765)))
                .setLinearHeadingInterpolation(Math.toRadians(36.3), Math.toRadians(0))
                .setVelocityConstraint(SLOW_INTAKE)
                .build();

        Loadup1stset = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(101.176, 80.765), new Pose(123.882, 80.294)))
                .setLinearHeadingInterpolation(0, 0)
                .setVelocityConstraint(SLOW_INTAKE)
                .build();
        Loadup1stsetBackup = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(122.882, 80.294), new Pose(119.882, 83.294)))
                .setLinearHeadingInterpolation(0, 0)
                .setVelocityConstraint(SLOW_INTAKE)
                .build();
        Loadup1stset3rdBall = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(119.882, 83.294), new Pose(130.882, 83.294)))
                .setLinearHeadingInterpolation(0, 0)
                .setVelocityConstraint(SLOW_INTAKE)
                .build();


        Shoot1stset = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(130.882, 83.294), new Pose(107.765, 92.471)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                .setVelocityConstraint(FAST_TRAVEL)
                .build();

        // ================= LOAD 2 =================
        AlligntoLoadup2ndset = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(75.765, 92.471), new Pose(101.176, 60.000)))
                .setLinearHeadingInterpolation(Math.toRadians(36.3), Math.toRadians(0))
                .setVelocityConstraint(SLOW_INTAKE)
                .build();

        Loadup2ndset = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(101.176, 60.000), new Pose(121.882, 60.000)))
                .setLinearHeadingInterpolation(0, 0)
                .setVelocityConstraint(SLOW_INTAKE)
                .build();

        Shoot2ndSet = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(121.882, 60.000), new Pose(75.765, 92.471)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(36.3))
                .setVelocityConstraint(FAST_TRAVEL)
                .build();

        // ================= LOAD 3 =================
        Alligntoloadup3rdset = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(75.765, 92.471), new Pose(101.176, 36.000)))
                .setLinearHeadingInterpolation(Math.toRadians(36.3), Math.toRadians(0))
                .setVelocityConstraint(SLOW_INTAKE)
                .build();

        Loadup3rdset = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(101.176, 36.000), new Pose(121.882, 36.000)))
                .setLinearHeadingInterpolation(0, 0)
                .setVelocityConstraint(SLOW_INTAKE)
                .build();

        Shoot3rdset = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(121.882, 36.000), new Pose(75.765, 92.471)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(36.3))
                .setVelocityConstraint(FAST_TRAVEL)
                .build();
    }
}
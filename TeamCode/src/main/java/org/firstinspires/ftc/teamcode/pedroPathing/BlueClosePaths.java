package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class BlueClosePaths {

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

    public BlueClosePaths(Follower follower) {

        // ================= PRELOAD =================
        Shootpreload = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(27.451, 130.431),
                        new Pose(36.235, 92.471)))
                .setLinearHeadingInterpolation(
                        Math.toRadians(-36.3),   // 180 - 216.3
                        Math.toRadians(135))     // 180 - 45
                .setVelocityConstraint(FAST_TRAVEL)
                .build();

        // ================= LOAD 1 =================
        AlligntoLoadup1stset = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(36.235, 92.471),
                        new Pose(42.824, 83.765)))
                .setLinearHeadingInterpolation(
                        Math.toRadians(143.7),   // 180 - 36.3
                        Math.toRadians(180))
                .setVelocityConstraint(SLOW_INTAKE)
                .build();

        Loadup1stset = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(42.824, 83.765),
                        new Pose(20.118, 83.294)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .setVelocityConstraint(SLOW_INTAKE)
                .build();

        Loadup1stsetBackup = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(21.118, 83.294),
                        new Pose(24.118, 84.294)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .setVelocityConstraint(SLOW_INTAKE)
                .build();

        Loadup1stset3rdBall = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(24.118, 84.294),
                        new Pose(13.118, 84.294)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .setVelocityConstraint(SLOW_INTAKE)
                .build();

        Shoot1stset = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(13.118, 86.294),
                        new Pose(36.235, 97.471)))
                .setLinearHeadingInterpolation(
                        Math.toRadians(180),
                        Math.toRadians(130))
                .setVelocityConstraint(FAST_TRAVEL)
                .build();

        // ================= LOAD 2 =================
        AlligntoLoadup2ndset = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(68.235, 92.471),
                        new Pose(42.824, 60.000)))
                .setLinearHeadingInterpolation(
                        Math.toRadians(143.7),
                        Math.toRadians(180))
                .setVelocityConstraint(SLOW_INTAKE)
                .build();

        Loadup2ndset = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(42.824, 60.000),
                        new Pose(22.118, 60.000)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .setVelocityConstraint(SLOW_INTAKE)
                .build();

        Shoot2ndSet = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(22.118, 60.000),
                        new Pose(68.235, 92.471)))
                .setLinearHeadingInterpolation(
                        Math.toRadians(180),
                        Math.toRadians(143.7))
                .setVelocityConstraint(FAST_TRAVEL)
                .build();

        // ================= LOAD 3 =================
        Alligntoloadup3rdset = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(68.235, 92.471),
                        new Pose(42.824, 36.000)))
                .setLinearHeadingInterpolation(
                        Math.toRadians(143.7),
                        Math.toRadians(180))
                .setVelocityConstraint(SLOW_INTAKE)
                .build();

        Loadup3rdset = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(42.824, 36.000),
                        new Pose(22.118, 36.000)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .setVelocityConstraint(SLOW_INTAKE)
                .build();

        Shoot3rdset = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(22.118, 36.000),
                        new Pose(68.235, 92.471)))
                .setLinearHeadingInterpolation(
                        Math.toRadians(180),
                        Math.toRadians(143.7))
                .setVelocityConstraint(FAST_TRAVEL)
                .build();
    }
}

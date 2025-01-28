package org.firstinspires.ftc.teamcode.opModes.auto

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.pedropathing.localization.Pose
import com.pedropathing.pathgen.BezierLine
import com.pedropathing.pathgen.PathBuilder
import com.pedropathing.pathgen.PathChain
import com.pedropathing.pathgen.Point
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import org.firstinspires.ftc.teamcode.opModes.MegiddoOpMode
import org.firstinspires.ftc.teamcode.subsystems.followerSubsystem
import org.firstinspires.ftc.teamcode.subsystems.followerSubsystem.follower
@Disabled
@Autonomous
class blueBasketPath :MegiddoOpMode() {
    val builder = PathBuilder()
    lateinit var telemetryDB: MultipleTelemetry


    fun build() : PathChain{
        return follower.pathBuilder()
            .addPath(
                // Line 1
                BezierLine(
                    Point(8.700, 95.100, Point.CARTESIAN),
                    Point(41.029, 84.983, Point.CARTESIAN)
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(180.0))
            .addPath(
                // Line 2
                BezierLine(
                    Point(41.029, 84.983, Point.CARTESIAN),
                    Point(10.059, 128.382, Point.CARTESIAN)
                )
            )
            .setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(310.0))
            .addPath(
                // Line 3
                 BezierLine (
                 Point (10.059, 128.382, Point.CARTESIAN
            ),
         Point (20.666, 123.882, Point.CARTESIAN)
        )
        )
        .setLinearHeadingInterpolation(Math.toRadians(310.0), Math.toRadians(0.0))
        .addPath(
            // Line 4
             BezierLine (
                     Point (20.666, 123.882, Point.CARTESIAN
        ),
         Point (10.059, 128.382, Point.CARTESIAN)
        )
        )
        .setLinearHeadingInterpolation(Math.toRadians(0.0), Math.toRadians(310.0))
        .addPath(
            // Line 5
             BezierLine (
                     Point (10.059, 128.382, Point.CARTESIAN
        ),
         Point (20.137, 128.382, Point.CARTESIAN)
        )
        )
        .setLinearHeadingInterpolation(Math.toRadians(310.0), Math.toRadians(0.0))
        .addPath(
            // Line 6
         BezierLine (
         Point (20.137, 128.382, Point.CARTESIAN
        ),
         Point (10.059, 128.382, Point.CARTESIAN)
        )
        )
        .setLinearHeadingInterpolation(Math.toRadians(0.0), Math.toRadians(310.0))
        .addPath(
            // Line 7
         BezierLine (
        Point (10.059, 128.382, Point.CARTESIAN
        ),
         Point (20.137, 131.029, Point.CARTESIAN)
        )
        )
        .setLinearHeadingInterpolation(Math.toRadians(310.0), Math.toRadians(20.0))
        .addPath(
            // Line 8
         BezierLine (
         Point (20.137, 131.029, Point.CARTESIAN
        ),
         Point (10.059, 128.382, Point.CARTESIAN)
        )
        )
        .setLinearHeadingInterpolation(Math.toRadians(20.0), Math.toRadians(310.0)).build()

    }

    override fun myInit() {
//        build()
//        follower.setStartingPose(Pose(8.700, 95.100, Math.toRadians(180.0)));
        telemetryDB = MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry())
    }

    override fun myStart() {
//        followerSubsystem.followPath(build()).schedule()
        follower.followPath(build())
    }

    override fun myLoop() {
        follower.telemetryDebug(telemetryDB)
        telemetryDB.update()
        follower.update()
    }
}
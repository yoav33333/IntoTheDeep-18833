package org.firstinspires.ftc.teamcode.opModes

import com.pedropathing.follower.Follower
import com.pedropathing.localization.Pose
import com.pedropathing.localization.localizers.ThreeWheelLocalizer
import com.pedropathing.pathgen.BezierCurve
import com.pedropathing.pathgen.BezierLine
import com.pedropathing.pathgen.PathBuilder
import com.pedropathing.pathgen.PathChain
import com.pedropathing.pathgen.Point
import com.pedropathing.util.Constants
import com.qualcomm.robotcore.hardware.Servo
import dev.frozenmilk.dairy.cachinghardware.CachingServo
import dev.frozenmilk.dairy.core.FeatureRegistrar
import dev.frozenmilk.dairy.core.util.OpModeLazyCell
import dev.frozenmilk.mercurial.commands.Command
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.commands.groups.Sequential
import dev.frozenmilk.mercurial.commands.util.Wait
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants
import org.firstinspires.ftc.teamcode.util.getDistance
import org.firstinspires.ftc.teamcode.util.marker
import org.firstinspires.ftc.teamcode.util.postDistenceMarker
import org.firstinspires.ftc.teamcode.util.postTimeMarker
import org.firstinspires.ftc.teamcode.util.preDistenceMarker
import kotlin.math.pow
import kotlin.math.sqrt

abstract class AutoBase (var side: Side) : MegiddoOpMode(){
    enum class Side{
        basket{
            override var startingPose = Pose(9.000, 87.882, 0.0)
        },
        chamber{
            override var startingPose = Pose(9.000, 56.118, 0.0)
        };
        abstract var startingPose: Pose
    }

    lateinit var follower: Follower
//    val follower: Follower by OpModeLazyCell {
//        Constants.setConstants(FConstants::class.java, LConstants::class.java)
//        val f = Follower(FeatureRegistrar.activeOpMode.hardwareMap)
//        (f.poseUpdater.localizer as ThreeWheelLocalizer).resetEncoders()
//        follower.setStartingPose(side.startingPose)
//        f
//    }
    final override fun preInit() {
        Constants.setConstants(FConstants::class.java, LConstants::class.java)
        follower = Follower(FeatureRegistrar.activeOpMode.hardwareMap)
        (follower.poseUpdater.localizer as ThreeWheelLocalizer).resetEncoders()
        follower.setStartingPose(side.startingPose)
        follower.setCurrentPoseWithOffset(side.startingPose)
    }

    abstract override fun myStart()
    fun followPath(chain: PathChain) =Lambda("follow-path-chain")
            .setInit { follower.followPath(chain, true) }
            .setExecute {
                follower.update()
//                follower.telemetryDebug(telemetry);
            }
            .setFinish { !follower.isBusy || follower.isRobotStuck }

    fun followPathExtra(chain: PathChain): Lambda {
        return Lambda("follow-path-chain")
            .setInterruptible(true)
            .setInit { follower.followPath(chain, true)
                markerScheduler.schedule()}
            .setExecute {
                follower.update()
                follower.telemetryDebug(telemetry);
            }
            .setFinish { (!follower.isBusy && markerList.isEmpty() && runningMarkerList.isEmpty()) || follower.isRobotStuck }
            .setEnd {
                markerScheduler.cancel()
            }
    }
    val markerList = mutableListOf<marker>()
    val runningMarkerList = mutableListOf<marker>()
    val markerScheduler = Lambda("delay-scheduler")
        .setFinish{false}
        .setExecute{
            for(marker in markerList){
                when(marker){
                    is preDistenceMarker -> {preDistance(marker)}
                    is postDistenceMarker -> {postDistance(marker)}
                    is postTimeMarker -> {postTime(marker)}
                }
            }
        }
    fun preDistance(marker: preDistenceMarker){
        if(getDistance(Point(follower.pose), follower.currentPath.controlPoints
                [follower.currentPath.controlPoints.lastIndex]) < marker.distance){
            markerList.remove(marker)
            runningMarkerList.add(marker)
            (marker.command as Lambda).addEnd{runningMarkerList.remove(marker)}.schedule()
        }
    }
    fun postDistance(marker: postDistenceMarker){
        if(getDistance(Point(follower.pose), follower.currentPath.controlPoints[0])
            < marker.distance){
            markerList.remove(marker)
            runningMarkerList.add(marker)
            (marker.command as Lambda).addEnd{runningMarkerList.remove(marker)}.schedule()
        }

    }
    fun postTime(marker: postTimeMarker){
        markerList.remove(marker)
        runningMarkerList.add(marker)
        Sequential(Wait(marker.time), (marker.command as Lambda)
            .addEnd{runningMarkerList.remove(marker)}).schedule()
    }
    fun postTimeCommand(command: Command, time: Double) = Lambda("postTimeCommand")
        .setInit{markerList.add(postTimeMarker(command, time))}
    fun postDistCommand(command: Command, time: Double) = Lambda("postDistCommand")
        .setInit{markerList.add(postDistenceMarker(command, time))}
    fun preDistCommand(command: Command, time: Double) = Lambda("preDistCommand")
        .setInit{markerList.add(preDistenceMarker(command, time))}
    fun runNonBlocking(command: Command) = Lambda("run-non-blocking")
        .setInit{command.schedule()}


    fun makeLinePath(startingPose: Pose, endingPose: Pose) = follower.pathBuilder().addPath(
            BezierLine(
                Point(startingPose),
                Point(endingPose)
            )
        ).setLinearHeadingInterpolation(startingPose.heading, endingPose.heading).build()

    fun makeCurvePath(vararg poses: Pose) : PathChain{
        val arr = ArrayList<Point>()
        poses.forEach { arr.add(Point(it)) }
        return follower.pathBuilder().addPath(
            BezierCurve(*arr.toTypedArray())
        ).setLinearHeadingInterpolation(poses[0].heading, poses[poses.lastIndex].heading).build()
    }


}
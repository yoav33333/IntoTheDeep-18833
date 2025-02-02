package org.firstinspires.ftc.teamcode.opModes.auto

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.pedropathing.localization.Pose
import com.pedropathing.pathgen.BezierLine
import com.pedropathing.pathgen.Path
import com.pedropathing.pathgen.PathChain
import com.pedropathing.pathgen.Point
import com.pedropathing.util.Timer
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.frozenmilk.mercurial.commands.groups.Parallel
import dev.frozenmilk.mercurial.commands.groups.Sequential
import dev.frozenmilk.mercurial.commands.util.Wait
import org.firstinspires.ftc.teamcode.commands.extendoCommand
import org.firstinspires.ftc.teamcode.opModes.MegiddoOpMode
import org.firstinspires.ftc.teamcode.subsystems.clawSubsystem
import org.firstinspires.ftc.teamcode.subsystems.deposit
import org.firstinspires.ftc.teamcode.subsystems.followerSubsystem
import org.firstinspires.ftc.teamcode.subsystems.linearSlides


@Autonomous
class delayBasket : MegiddoOpMode() {
    var tele: MultipleTelemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
    //    private var follower: Follower? = null
    private var pathTimer: Timer? = null
    private val actionTimer: Timer? = null
    private var opmodeTimer: Timer? = null

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method.  */
    private var pathState = 0

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */
    /** Start Pose of our robot  */
    private val startPose = Pose(9.000, 87.882, Math.toRadians(0.0))
//    private val startPose2 = Pose(9.000, 87.882, Math.toRadians(180.0))

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle.  */
    private val ChamberPose = Pose(36.04117647058824, 82.412, Math.toRadians(180.0))

    /** Lowest (First) Sample from the Spike Mark  */
    private val pickup1Pose = Pose(19.28823529411765, 118.70588235294117, Math.toRadians(0.0))
    val basketScore = Pose(16.059, 126.382, Math.toRadians(310.0))

    /** Middle (Second) Sample from the Spike Mark  */
//    private val pickup2PoseExtra = Pose(24.737, 128.382, Math.toRadians(0.0))
    private val pickup2Pose = Pose(20.737, 129.382, Math.toRadians(0.0))
    val parkPose = Pose(59.82352941176471, 102.59823529, 0.0)
//
//    /** Highest (Third) Sample from the Spike Mark  */
//    private val pickup3Pose = Pose(49.0, 135.0, Math.toRadians(0.0))
//
//    /** Park Pose for our robot, after we do all of the scoring.  */
//    private val parkPose = Pose(60.0, 98.0, Math.toRadians(90.0))
//
//    /** Park Control Pose for our robot, this is used to manipulate the bezier curve that we will create for the parking.
//     * The Robot will not go to this pose, it is used a control point for our bezier curve.  */
//    private val parkControlPose = Pose(60.0, 98.0, Math.toRadians(90.0))

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private var scorePreload: PathChain? = null
    private var scorePreload2: PathChain? = null
    private var park: PathChain? = null
    private var grabPickup1: PathChain? = null
    private var grabPickup2Extra: PathChain? = null
    private var grabPickup2: PathChain? = null
    private var grabPickup3: PathChain? = null
    private var scorePickup1: PathChain? = null
    private var scorePickup2: PathChain? = null
    private var scorePickup3: PathChain? = null

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts.  */
    fun buildPaths() {

        scorePreload = followerSubsystem.follower.pathBuilder()
            .addPath(BezierLine(Point(startPose), Point(ChamberPose)))
            .setLinearHeadingInterpolation(startPose.heading, ChamberPose.heading)
            .build()
//        scorePreload2 = followerSubsystem.follower.pathBuilder()
//            .addPath(BezierLine(Point(ChamberPose), Point(startPose2)))
//            .setLinearHeadingInterpolation(ChamberPose.heading, startPose2.heading)
//            .build()
        grabPickup1 = followerSubsystem.follower.pathBuilder()
            .addPath(BezierLine(Point(ChamberPose), Point(pickup1Pose)))
            .setLinearHeadingInterpolation(ChamberPose.heading, pickup1Pose.heading)
            .build()

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = followerSubsystem.follower.pathBuilder()
            .addPath(BezierLine(Point(pickup1Pose), Point(basketScore)))
            .setLinearHeadingInterpolation(pickup1Pose.heading, basketScore.heading)
            .build()
        park = followerSubsystem.follower.pathBuilder()
            .addPath(BezierLine(Point(basketScore), Point(parkPose)))
            .setLinearHeadingInterpolation(basketScore.heading, parkPose.heading)
            .build()

//        grabPickup2 = followerSubsystem.follower.pathBuilder()
//            .addPath(BezierLine(Point(basketScore), Point(pickup2Pose)))
//            .setLinearHeadingInterpolation(basketScore.heading, pickup2Pose.heading)
//            .build()
////        grabPickup2 = followerSubsystem.follower.pathBuilder()
////            .addPath(BezierLine(Point(pickup2PoseExtra), Point(pickup2PoseExtra)))
////            .setLinearHeadingInterpolation(pickup2PoseExtra.heading, pickup2PoseExtra.heading)
////            .build()
//
//        scorePickup2 = followerSubsystem.follower.pathBuilder()
//            .addPath(BezierLine(Point(pickup2Pose), Point(basketScore)))
//            .setLinearHeadingInterpolation(pickup2Pose.heading, basketScore.heading)
//            .build()

    }


    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches  */
    fun setPathState(pState: Int) {
        pathState = pState
        pathTimer!!.resetTimer()
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play".  */
    override fun myLoop() {
        // These loop the movements of the robot

//        autonomousPathUpdate()

        followerSubsystem.follower.update()
//        linearSlides.runToPose(linearSlides.target)
        // Feedback to Driver Hub
//        telemetry.addData("path state", pathState)
        tele.addData("x", followerSubsystem.follower.pose.x)
        tele.addData("y", followerSubsystem.follower.pose.y)
        tele.addData("heading", followerSubsystem.follower.pose.heading)
        tele.addData("x offset", followerSubsystem.follower.xOffset)
        tele.addData("y offset", followerSubsystem.follower.yOffset)
        tele.addData("heading offset", followerSubsystem.follower.headingOffset)

        tele.update()
    }

    /** This method is called once at the init of the OpMode.  */
    override fun myInit() {
        clawSubsystem.clawRotationServo.position = 0.5
        deposit.closeClaw()
        deposit.armOutHalf()

        pathTimer = Timer()
        opmodeTimer = Timer()
        opmodeTimer!!.resetTimer()
        followerSubsystem.follower.setCurrentPoseWithOffset(startPose)
//        followerSubsystem.follower.setStartingPose(startPose)


//        Constants.setConstants(FConstants::class.java, LConstants::class.java)

//        follower = followerSubsystem.follower
//        followerSubsystem.follower.setCurrentPoseWithOffset(startPose)
        buildPaths()
    }

    /** This method is called continuously after Init while waiting for "play".  */
    override fun myInitLoop() {
        clawSubsystem.clawRotationServo.position = 0.5
        tele.addData("x", followerSubsystem.follower.pose.x)
        tele.addData("y", followerSubsystem.follower.pose.y)
        tele.addData("heading", followerSubsystem.follower.pose.heading)
        tele.addData("x offset", followerSubsystem.follower.xOffset)
        tele.addData("y offset", followerSubsystem.follower.yOffset)
        tele.addData("heading offset", followerSubsystem.follower.headingOffset)

        tele.update()
        followerSubsystem.follower.update()
    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system  */
    override fun myStart() {
        deposit.armOut()
        followerSubsystem.follower.setMaxPower(0.0)
        followerSubsystem.follower.update()
        followerSubsystem.follower.setCurrentPoseWithOffset(startPose)
        followerSubsystem.follower.setMaxPower(0.85)
//        followerSubsystem.follower.followPath(scorePreload)
        opmodeTimer!!.resetTimer()
//        setPathState(0)
        linearSlides.runToPosition.schedule()
        Sequential(
            Wait(3.0),
            extendoCommand.extendoReset,
            linearSlides.goToHighChamber,
            followerSubsystem.followPathChain(scorePreload!!),
            Wait(0.2),
            deposit.slamSeq,
            Wait(0.2),
            Parallel(
                followerSubsystem.followPathChain(grabPickup1!!),
                Sequential(
                    Wait(0.5),
                    extendoCommand.extendoOpenCommand
                )

            ),
            followerSubsystem.followPathChain(grabPickup1!!),
            Wait(1.0),
            clawSubsystem.closeClaw,
            Wait(0.7),
            Parallel(
                extendoCommand.extendoCloseCommandAuto,
                linearSlides.goToHighBasket,
            ),
            Wait(2.0),
            followerSubsystem.followPath(scorePickup1!!),
            Wait(0.1),
            deposit.release,
            Wait(0.2),
            followerSubsystem.followPath(park!!),
            linearSlides.goToLowChamber
//            followerSubsystem.followPathChain(grabPickup2Extra!!),
//            followerSubsystem.followPathChain(grabPickup2!!),
//            extendoCommand.extendoOpenCommand,
//            Wait(1.0),
//            clawSubsystem.closeClaw,
//            Wait(0.7),
//            Parallel(
//                extendoCommand.extendoCloseCommandAuto,
//                linearSlides.goToHighBasket,
//                followerSubsystem.followPath(scorePickup2!!)
//            ),
//            Wait(0.7),
//            clawSubsystem.openClaw

        ).schedule()
    }

    override fun myStop() {
        followerSubsystem.follower.setCurrentPoseWithOffset(Pose(0.0,0.0,0.0))
    }
}
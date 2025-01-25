package org.firstinspires.ftc.teamcode.opModes.auto

import com.pedropathing.follower.Follower
import com.pedropathing.localization.Encoder
import com.pedropathing.localization.Pose
import com.pedropathing.pathgen.BezierLine
import com.pedropathing.pathgen.Path
import com.pedropathing.pathgen.PathChain
import com.pedropathing.pathgen.Point
import com.pedropathing.util.Constants
import com.pedropathing.util.Timer
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import dev.frozenmilk.mercurial.commands.groups.Sequential
import dev.frozenmilk.mercurial.commands.util.Wait
import org.firstinspires.ftc.teamcode.commands.extendoCommand
import org.firstinspires.ftc.teamcode.opModes.CommandOpMode
import org.firstinspires.ftc.teamcode.opModes.MegiddoOpMode
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants
import org.firstinspires.ftc.teamcode.subsystems.clawSubsystem
import org.firstinspires.ftc.teamcode.subsystems.deposit
import org.firstinspires.ftc.teamcode.subsystems.followerSubsystem
import org.firstinspires.ftc.teamcode.subsystems.linearSlides


@Autonomous
class simpleBasket : MegiddoOpMode() {
//    var tele: MultipleTelemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
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
    private val startPose = Pose(9.000, 87.882, Math.toRadians(180.0))

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle.  */
    private val scorePose = Pose(36.94117647058824, 84.412, Math.toRadians(180.0))

    /** Lowest (First) Sample from the Spike Mark  */
    private val pickup1Pose = Pose(22.08823529411765, 118.70588235294117, Math.toRadians(0.0))

    /** Middle (Second) Sample from the Spike Mark  */
//    private val pickup2Pose = Pose(43.0, 130.0, Math.toRadians(0.0))
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
    private var park: Path? = null
    private var grabPickup1: PathChain? = null
    private var grabPickup2: PathChain? = null
    private var grabPickup3: PathChain? = null
    private var scorePickup1: PathChain? = null
    private var scorePickup2: PathChain? = null
    private var scorePickup3: PathChain? = null

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts.  */
    fun buildPaths() {

        scorePreload = followerSubsystem.follower.pathBuilder()
            .addPath(BezierLine(Point(startPose), Point(scorePose)))
            .setLinearHeadingInterpolation(startPose.heading, scorePose.heading)
            .build()
        grabPickup1 = followerSubsystem.follower.pathBuilder()
            .addPath(BezierLine(Point(scorePose), Point(pickup1Pose)))
            .setLinearHeadingInterpolation(scorePose.heading, pickup1Pose.heading)
            .build()

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = followerSubsystem.follower.pathBuilder()
            .addPath(BezierLine(Point(pickup1Pose), Point(scorePose)))
            .setLinearHeadingInterpolation(pickup1Pose.heading, scorePose.heading)
            .build()

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

        followerSubsystem.follower.update()
//        autonomousPathUpdate()

//        followerSubsystem.follower.telemetryDebug(tele)
        linearSlides.runToPose(linearSlides.target)
        // Feedback to Driver Hub
//        telemetry.addData("path state", pathState)
        telemetry.addData("x", followerSubsystem.follower.pose.x)
        telemetry.addData("y", followerSubsystem.follower.pose.y)
        telemetry.addData("heading", followerSubsystem.follower.pose.heading)
        telemetry.update()
    }

    /** This method is called once at the init of the OpMode.  */
    override fun myInit() {
        deposit.closeClaw()
        deposit.armOut()
        pathTimer = Timer()
        opmodeTimer = Timer()
        opmodeTimer!!.resetTimer()

//        Constants.setConstants(FConstants::class.java, LConstants::class.java)
        hardwareMap.get(DcMotorEx::class.java, "dfl").mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        hardwareMap.get(DcMotorEx::class.java, "dfl").mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        hardwareMap.get(DcMotorEx::class.java, "drl").mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        hardwareMap.get(DcMotorEx::class.java, "drl").mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        hardwareMap.get(DcMotorEx::class.java, "drr").mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        hardwareMap.get(DcMotorEx::class.java, "drr").mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
//        follower = followerSubsystem.follower
        followerSubsystem.follower.setCurrentPoseWithOffset(startPose)
//        followerSubsystem.follower.setCurrentPoseWithOffset(startPose)
        buildPaths()
    }

    /** This method is called continuously after Init while waiting for "play".  */
    override fun myInitLoop() {

        telemetry.addData("x", followerSubsystem.follower.pose.x)
        telemetry.addData("y", followerSubsystem.follower.pose.y)
        telemetry.addData("heading", followerSubsystem.follower.pose.heading)
        telemetry.update()
    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system  */
    override fun myStart() {
        followerSubsystem.follower.setCurrentPoseWithOffset(startPose)

        opmodeTimer!!.resetTimer()
        setPathState(0)
        Sequential(
            linearSlides.goToHighChamber,
            followerSubsystem.followPath(scorePreload!!),
            Wait(0.2),
            deposit.slamSeq,
            Wait(0.2),
            extendoCommand.extendoOpenCommand,
            followerSubsystem.followPath(grabPickup1!!),
            Wait(0.2),
            clawSubsystem.closeClaw,
            Wait(0.3),
            extendoCommand.extendoCloseCommand

        ).schedule()
    }
}
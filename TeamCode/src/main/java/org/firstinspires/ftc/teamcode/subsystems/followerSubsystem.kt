package org.firstinspires.ftc.teamcode.subsystems


import com.acmerobotics.dashboard.config.Config
import com.pedropathing.follower.Follower
import com.pedropathing.localization.Pose
import com.pedropathing.pathgen.BezierCurve
import com.pedropathing.pathgen.BezierLine
import com.pedropathing.pathgen.PathChain
import com.pedropathing.pathgen.Point
import com.qualcomm.robotcore.hardware.Gamepad
import dev.frozenmilk.dairy.core.FeatureRegistrar
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.util.OpModeLazyCell
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.subsystems.SDKSubsystem
import dev.frozenmilk.mercurial.subsystems.Subsystem
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants
import java.lang.annotation.Inherited

@Config
object followerSubsystem : SDKSubsystem() {
    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and
            SingleAnnotation(Mercurial.Attach::class.java)

    @Target(AnnotationTarget.CLASS)
    @Retention(AnnotationRetention.RUNTIME)
    @MustBeDocumented
    @Inherited
    annotation class Attach
//    var isCentric = true
    val gamepad1: Gamepad by OpModeLazyCell {
        FeatureRegistrar.activeOpMode.gamepad1
    }
    val gamepad2: Gamepad by OpModeLazyCell {
        FeatureRegistrar.activeOpMode.gamepad2
    }
    @JvmStatic
    var startingPose = Pose(0.0,3.0,0.0)
//    var startingPose2 = Pose(0.0,3.0,0.0)
    lateinit var
            follower: Follower
    /**
     * Initializes the follower instance with hardware and constants before user initialization.
     *
     * Sets up the path-following subsystem by creating a new `Follower` using the hardware map and constants classes.
     */
    override fun preUserInitHook(opMode: Wrapper) {
        //        Constants.setConstants(FConstants.class, LConstants.class);
        follower = Follower(hardwareMap, FConstants::class.java, LConstants::class.java)
        //                Constants.setConstants(FConstants.class, LConstants.class);
//        startingPose2 = startingPose
//        follower.poseUpdater.pose = startingPose2

//        (follower.poseUpdater.localizer as ThreeWheelIMULocalizer).resetEncoders()
//        isCentric = false
    }

    /**
     * Hook called after the user op mode starts.
     *
     * Currently, this function does not perform any actions.
     */
    override fun postUserStartHook(opMode: Wrapper) {
//        follower.poseUpdater.setPose(startingPose)
    }
//    val changeCentric = Lambda("CC")
//        .setInit{ isCentric = !isCentric}
//    val runRobotCentric = Lambda("RBC")
//        .setInit{ isCentric = true}
//        .setFinish{false}
//        .setEnd{ isCentric = false}
    val runFollower = Lambda("runFollower")
        .setExecute{ follower.update()}
        .setFinish{false}
    fun followPath(path: PathChain) = Lambda("follow-path")
        .setInit {
            follower.followPath(path, true)
        }
        .setFinish {
            !follower.isBusy
        }


    fun followPathChain(chain: PathChain?): Lambda {
        return Lambda("follow-path-chain")
            .setInterruptible(true)
            .setInit { follower.followPath(chain, true) }
            .setExecute {
                follower.update()
                follower.telemetryDebug(telemetry);
            }
            .setFinish { !follower.isBusy || follower.isRobotStuck }
            .setEnd { interrupted: Boolean ->
                if (interrupted) follower.breakFollowing();
            }
    }
    val firstGear = Lambda("g1")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit { follower.setMaxPower(1.0) }
    val secondGear = Lambda("g2")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit { follower.setMaxPower(0.40) }
    val thirdGear = Lambda("g3")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit { follower.setMaxPower(0.35) }
    val angleReset = Lambda("angleReset")
        .setInit{ follower.headingOffset = -follower.totalHeading
            follower.headingOffset = -follower.totalHeading}
    /**
          * Creates a Lambda that sets the follower's heading offset to align the robot's heading with the specified value.
          *
          * @param heading The desired heading in radians to set as the new reference.
          * @return A Lambda that, when initialized, updates the follower's heading offset.
          */
         fun setHeading(heading: Double) = Lambda("angleReset")
        .setInit{ follower.headingOffset = -(follower.totalHeading-heading)
         follower.headingOffset = -(follower.totalHeading-heading)}
    @JvmField
    var headingPow = 0.5
    /**
     * Initializes teleop drive mode by starting the follower's teleop drive, setting maximum power, and applying initial movement vectors based on gamepad inputs.
     *
     * This method prepares the robot for manual control, scaling heading input by `headingPow` and combining trigger and stick values from both gamepads to determine movement.
     */
    fun initDrive(){
//        follower.poseUpdater.setPose(startingPose)
        follower.startTeleopDrive()
//            follower.setTeleOpMovementVectors(0.5,0.0,0.0)
//            follower.update()
//            follower.setTeleOpMovementVectors(0.0,0.0,0.0)
//            follower.update()
//            follower.setCurrentPoseWithOffset(startingPose)
        follower.setMaxPower(1.0)
        follower.setTeleOpMovementVectors(-(gamepad1.left_stick_y + (gamepad1.left_trigger) - gamepad1.right_trigger).toDouble(),
            -(gamepad1.left_stick_x).toDouble(),
            -headingPow*(gamepad1.right_stick_x + 0.65*(gamepad2.right_trigger - gamepad2.left_trigger))
            , gamepad1.left_trigger+ gamepad1.right_trigger>0.1
        )
        follower.update()
//        follower.poseUpdater.setPose(startingPose)
//        follower.poseUpdater.setPose(startingPose)
//        Sequential(Wait(0.1), InstantCommand{
//        follower.headingOffset = -startingPose.heading
//        follower.yOffset = -startingPose.y
//        follower.xOffset = -startingPose.x}).schedule()
//        follower.poseUpdater.localizer.pose = startingPose
    }

    val teleopDrive = Lambda("teleop-drive")
        .setInit {
//            follower.setCurrentPoseWithOffset(Pose(0.0, 0.0, 0.0)) // Reset pose at the start of the path
            //Constants.setConstants(FConstants.class, LConstants.class);

            initDrive()
            setHeading(startingPose.heading)

        }
        .setExecute {
            follower.setTeleOpMovementVectors(-(gamepad1.left_stick_y + (gamepad1.left_trigger) - gamepad1.right_trigger).toDouble(),
                -(gamepad1.left_stick_x).toDouble(),
                -headingPow*(gamepad1.right_stick_x + 0.65*(gamepad2.right_trigger - gamepad2.left_trigger))
                , gamepad1.left_trigger+ gamepad1.right_trigger>0.1
            )
//            follower.telemetryDebug(MultipleTelemetry(FtcDashboard.getInstance().telemetry, telemetry))
            follower.update()

        }
        .setFinish { false }
    //        .setEnd{
//            startingPose = follower.pose
//        }
    val forward = Lambda("forward")
        .setInit {
            follower.startTeleopDrive()
            follower.setMaxPower(1.0)
        }
        .setExecute{
           follower.setTeleOpMovementVectors(-1.0,
            0.0,
            0.0)
            follower.update()}
        .setFinish{false}
    val stop = Lambda("stop")
        .setInit{ forward.cancel()
            follower.setTeleOpMovementVectors(0.0,
                0.0,
                0.0)
            follower.update()
        }
    fun makeLinePath(startingPose: Pose, endingPose: Pose) = follower.pathBuilder().addPath(
            BezierLine(
            Point(startingPose),
            Point(endingPose)
            )
        ).setLinearHeadingInterpolation(startingPose.heading, endingPose.heading).build()
    /**
     * Creates a Bezier curve path through the given poses with linear heading interpolation from the first to the last pose.
     *
     * @param poses Sequence of poses defining the control points of the curve.
     * @return A PathChain representing the constructed Bezier curve path.
     */
    fun makeCurvePath(vararg poses: Pose) : PathChain{
        val arr = ArrayList<Point>()
        poses.forEach { arr.add(Point(it)) }
        return follower.pathBuilder().addPath(
            BezierCurve(*arr.toTypedArray())
        ).setLinearHeadingInterpolation(poses[0].heading, poses[poses.lastIndex].heading).build()
    }

}

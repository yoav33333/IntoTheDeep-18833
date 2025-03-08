package org.firstinspires.ftc.teamcode.subsystems


import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.pedropathing.follower.Follower
import com.pedropathing.localization.Pose
import com.pedropathing.pathgen.BezierCurve
import com.pedropathing.pathgen.BezierLine
import com.pedropathing.pathgen.PathChain
import com.pedropathing.pathgen.Point
import com.pedropathing.util.Constants
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
import org.firstinspires.ftc.teamcode.opModes.AutoBaseJava
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants
import org.firstinspires.ftc.teamcode.util.FollowerInstance
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
    var isCentric = true
    val gamepad1: Gamepad by OpModeLazyCell {
        FeatureRegistrar.activeOpMode.gamepad1
    }
    val gamepad2: Gamepad by OpModeLazyCell {
        FeatureRegistrar.activeOpMode.gamepad2
    }
    lateinit var follower: Follower
    override fun preUserInitHook(opMode: Wrapper) {
        Constants.setConstants(FConstants::class.java, LConstants::class.java)
//        FollowerInstance.reset(FeatureRegistrar.activeOpMode.hardwareMap)
        follower = FollowerInstance.getInstance(FeatureRegistrar.activeOpMode.hardwareMap)
    }
    val changeCentric = Lambda("CC")
        .setInit{ isCentric = !isCentric}
    val runRobotCentric = Lambda("RBC")
        .setInit{ isCentric = true}
        .setFinish{false}
        .setEnd{ isCentric = false}
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
        .setInit{ follower.headingOffset = -follower.totalHeading}
    @JvmField
    var headingPow = 0.5
    val teleopDrive = Lambda("teleop-drive")
        .setInit {
            follower.startTeleopDrive()
//            follower.setTeleOpMovementVectors(0.5,0.0,0.0)
//            follower.update()
//            follower.setTeleOpMovementVectors(0.0,0.0,0.0)
//            follower.update()
//            follower.setCurrentPoseWithOffset(startingPose)
            follower.setMaxPower(1.0)
        }
        .setExecute {
            follower.setTeleOpMovementVectors(-(gamepad1.left_stick_y - (gamepad1.left_trigger)).toDouble(),
                -(gamepad1.left_stick_x).toDouble(),
                -headingPow*(gamepad1.right_stick_x + 0.3*(gamepad2.right_trigger - gamepad2.left_trigger))*(if (gamepad1.right_trigger>0.1) 0.22 else 1.0)
                , gamepad1.left_trigger>0.1|| isCentric
            )
//            follower.telemetryDebug(MultipleTelemetry(FtcDashboard.getInstance().telemetry, telemetry))
            follower.update()
        }
        .setFinish { false }
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
    fun makeCurvePath(vararg poses: Pose) : PathChain{
        val arr = ArrayList<Point>()
        poses.forEach { arr.add(Point(it)) }
        return follower.pathBuilder().addPath(
            BezierCurve(*arr.toTypedArray())
        ).setLinearHeadingInterpolation(poses[0].heading, poses[poses.lastIndex].heading).build()
    }


}
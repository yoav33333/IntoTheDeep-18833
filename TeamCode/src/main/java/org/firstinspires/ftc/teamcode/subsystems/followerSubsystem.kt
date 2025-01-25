package org.firstinspires.ftc.teamcode.subsystems


import com.pedropathing.follower.Follower
import com.pedropathing.localization.Pose
import com.pedropathing.pathgen.BezierCurve
import com.pedropathing.pathgen.BezierLine
import com.pedropathing.pathgen.PathBuilder
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
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants
import java.lang.annotation.Inherited


object followerSubsystem : SDKSubsystem() {
    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and
            SingleAnnotation(Mercurial.Attach::class.java)

    @Target(AnnotationTarget.CLASS)
    @Retention(AnnotationRetention.RUNTIME)
    @MustBeDocumented
    @Inherited
    annotation class Attach

    val gamepad1: Gamepad by OpModeLazyCell {
        FeatureRegistrar.activeOpMode.gamepad1
    }
    val gamepad2: Gamepad by OpModeLazyCell {
        FeatureRegistrar.activeOpMode.gamepad2
    }
    lateinit var follower: Follower
    override fun preUserInitHook(opMode: Wrapper) {
        Constants.setConstants(FConstants::class.java, LConstants::class.java)
        follower = Follower(FeatureRegistrar.activeOpMode.hardwareMap)

    }
    fun setStartingPose(pose: Pose){
//        follower.setCurrentPoseWithOffset(pose)
        follower.setStartingPose(pose)
    }
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

    val firstGear = Lambda("g1")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit { follower.setMaxPower(1.0) }
    val secondGear = Lambda("g2")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit { follower.setMaxPower(0.65) }
    val thirdGear = Lambda("g3")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit { follower.setMaxPower(0.35) }
    val angleReset = Lambda("angleReset")
        .setInit{ follower.headingOffset = -follower.totalHeading}
    val teleopDrive = Lambda("teleop-drive")
        .setInit {
            follower.startTeleopDrive()
            follower.setMaxPower(1.0)
        }
        .setExecute {
            follower.setTeleOpMovementVectors(-(gamepad1.left_stick_y + (if(gamepad1.dpad_down) 0.3 else 0.0)).toDouble(),
                -(gamepad1.left_stick_x - (if (gamepad1.right_bumper) 1 else 0 - if (gamepad1.left_bumper) 1 else 0)).toDouble(),
                -(gamepad1.right_stick_x +gamepad1.right_trigger - gamepad1.left_trigger  + 0.3*(gamepad2.right_trigger - gamepad2.left_trigger)).toDouble()
                ,false
            )
            follower.update()
        }
        .setFinish { false }
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
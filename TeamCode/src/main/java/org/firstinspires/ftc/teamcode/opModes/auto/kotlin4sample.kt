package org.firstinspires.ftc.teamcode.opModes.auto

import com.acmerobotics.dashboard.config.Config
import com.pedropathing.localization.Pose
import com.pedropathing.pathgen.PathChain
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.frozenmilk.mercurial.commands.groups.Parallel
import dev.frozenmilk.mercurial.commands.groups.Sequential
import dev.frozenmilk.mercurial.commands.util.Wait
import org.firstinspires.ftc.teamcode.commands.extendoCommand.extendoCloseCommandAuto
import org.firstinspires.ftc.teamcode.commands.extendoCommand.extendoOpenCommandAuto
import org.firstinspires.ftc.teamcode.commands.extendoCommand.extendoReset
import org.firstinspires.ftc.teamcode.opModes.AutoBaseJava
import org.firstinspires.ftc.teamcode.opModes.idk
import org.firstinspires.ftc.teamcode.subsystems.clawSubsystem.clawRotationServo
import org.firstinspires.ftc.teamcode.subsystems.clawSubsystem.clawServo
import org.firstinspires.ftc.teamcode.subsystems.clawSubsystem.closeClaw
import org.firstinspires.ftc.teamcode.subsystems.deposit.armIn
import org.firstinspires.ftc.teamcode.subsystems.deposit.armOutBasket
import org.firstinspires.ftc.teamcode.subsystems.deposit.armOutHalf
import org.firstinspires.ftc.teamcode.subsystems.deposit.closeClawRaw
import org.firstinspires.ftc.teamcode.subsystems.deposit.release
import org.firstinspires.ftc.teamcode.subsystems.linearSlides.getPose
import org.firstinspires.ftc.teamcode.subsystems.linearSlides.goToHighBasket
import org.firstinspires.ftc.teamcode.subsystems.linearSlides.runToPosition
import org.firstinspires.ftc.teamcode.subsystems.linearSlides.setPose
import org.firstinspires.ftc.teamcode.subsystems.linearSlides.touchBar

@Config
@Autonomous
class kotlin4sample : idk(Side.basket) {
    override fun myInit() {
        scorePreload = makeLinePath(startPose, basketPose1)
        pickup1 = makeLinePath(basketPose1, pickup1Pose)
        scorePickup1 = makeLinePath(pickup1Pose, basketPose2)
        pickup2 = makeLinePath(basketPose2, pickup2Pose)
        scorePickup2 = makeLinePath(pickup2Pose, basketPose3)
        pickup3 = makeLinePath(basketPose3, pickup3Pose)
        scorePickup3 = makeLinePath(pickup3Pose, basketPose4)
        park = makeCurvePath(basketPose4, parkControl, parkPose)

        clawRotationServo.position = 0.5
        clawServo.position = 0.0
        armOutHalf()
        closeClawRaw()
    }


    override fun myStart() {
        setPose(0)
        runToPosition.cancel()
        runToPosition.schedule()
        Sequential(
            extendoReset,
            goToHighBasket,
            waitUntil { getPose() > 600 },
            armOutBasket,
            waitUntil { getPose() > 2000 },
            followPath(scorePreload),
            waitUntil { getPose() > 3400 },
            release,
            Wait(0.1),
            Parallel(
                Sequential(
                    waitUntil { follower!!.currentTValue > 0.1 },
                    extendoOpenCommandAuto
                ),
                followPath(pickup1)
            ),
            Wait(0.2),
            closeClaw,
            Wait(0.2),
            goToHighBasket,
            extendoCloseCommandAuto,
            waitUntil { getPose() > 2400 },
            followPath(scorePickup1),
            waitUntil { getPose() > 3400 },
            release,
            Wait(0.1),
            Parallel(
                extendoOpenCommandAuto,
                followPath(pickup2)
            ),
            Wait(0.3),
            closeClaw,
            Wait(0.2),
            goToHighBasket,
            extendoCloseCommandAuto,
            waitUntil { getPose() > 2400 },
            followPath(scorePickup2),
            waitUntil { getPose() > 3400 },
            release,
            Wait(0.1),
            Parallel(
                extendoOpenCommandAuto,
                followPath(pickup3)
            ),
            Wait(0.3),
            closeClaw,
            Wait(0.2),
            goToHighBasket,
            extendoCloseCommandAuto,
            waitUntil { getPose() > 2200 },
            followPath(scorePickup3),
            waitUntil { getPose() > 3400 },
            release,
            Wait(0.1),
            Parallel(
                followPath(park),
                Sequential(
                    waitUntil { follower!!.currentTValue > 0.3 },
                    touchBar,
                    armIn
                )
            ),

            finishAuto
        ).schedule()
    }

    companion object {
        var startPose: Pose = startingPoseBasket
        var basketPose1: Pose = Pose(-59.0, 53.0, Math.toRadians(-45.0))
        var basketPose2: Pose = Pose(-56.5, 55.0, Math.toRadians(-45.0))
        var basketPose3: Pose = Pose(-56.5, 57.0, Math.toRadians(-45.0))
        var basketPose4: Pose = Pose(-55.0, 58.0, Math.toRadians(-45.0))
        var pickup1Pose: Pose = Pose(-53.0, 48.3, 0.0)
        var pickup2Pose: Pose = Pose(-53.0, 58.0, Math.toRadians(0.0))
        var pickup3Pose: Pose = Pose(-51.8, 59.2, Math.toRadians(20.0))
        var parkPose: Pose = Pose(-7.8, 24.0, Math.toRadians(90.0))
        var parkControl: Pose = Pose(-9.0, 55.0, Math.toRadians(0.0))

        var scorePreload: PathChain? = null
        var scorePickup1: PathChain? = null
        var scorePickup2: PathChain? = null
        var scorePickup3: PathChain? = null
        var pickup1: PathChain? = null
        var pickup2: PathChain? = null
        var pickup3: PathChain? = null
        var park: PathChain? = null
    }
}
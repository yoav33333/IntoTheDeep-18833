package org.firstinspires.ftc.teamcode.opModes.auto

import com.pedropathing.localization.Pose
import com.pedropathing.pathgen.PathChain
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.frozenmilk.mercurial.commands.groups.Parallel
import dev.frozenmilk.mercurial.commands.groups.Sequential
import dev.frozenmilk.mercurial.commands.util.Wait
import org.firstinspires.ftc.teamcode.commands.extendoCommand
import org.firstinspires.ftc.teamcode.commands.extendoCommand.extendoOpenCommand
import org.firstinspires.ftc.teamcode.opModes.AutoBase
import org.firstinspires.ftc.teamcode.subsystems.clawSubsystem
import org.firstinspires.ftc.teamcode.subsystems.deposit
import org.firstinspires.ftc.teamcode.subsystems.followerSubsystem
import org.firstinspires.ftc.teamcode.subsystems.linearSlides

@Autonomous
class newAuto : AutoBase(Side.basket) {

    //poses
    val startPose = side.startingPose
    val chamberPose = Pose(36.04117647058824, 82.412, Math.toRadians(180.0))
    val pickup1Pose = Pose(19.28823529411765, 118.70588235294117, Math.toRadians(0.0))
    val basketScore = Pose(16.059, 126.382, Math.toRadians(310.0))
    val pickup2Pose = Pose(20.737, 129.382, Math.toRadians(0.0))
    val parkPose = Pose(59.82352941176471, 102.59823529, 0.0)

    //paths
    lateinit var scorePreload : PathChain
    lateinit var grabPickup1 : PathChain
    lateinit var scorePickup1 : PathChain
    lateinit var park : PathChain
    override fun myInit() {
        scorePreload = makeLinePath(startPose, chamberPose)
        grabPickup1 = makeLinePath(chamberPose, pickup1Pose)
        scorePickup1 = makeLinePath(pickup1Pose, basketScore)
        park = makeLinePath(basketScore, parkPose)
        clawSubsystem.clawRotationServo.position = 0.5
        deposit.closeClaw()
        deposit.armOutHalf()
    }
    override fun myStart() {
        deposit.armOut()

        linearSlides.runToPosition.schedule()
        Sequential(
            extendoCommand.extendoReset,
            linearSlides.goToHighChamber,
            followPath(scorePreload),
            Wait(0.2),
            deposit.slamSeq,
            Wait(0.2),
            Parallel(
                followPath(grabPickup1),
                Sequential(
                    Wait(0.5),
                    extendoOpenCommand
                )
            ),
            followPath(grabPickup1),
            Wait(1.0),
            clawSubsystem.closeClaw,
            Wait(1.2),
            Parallel(
                extendoCommand.extendoCloseCommandAuto,
                linearSlides.goToHighBasket,
            ),
            Wait(2.0),
            followPath(scorePickup1!!),
            Wait(0.1),
            deposit.release,
            Wait(0.2),
            followPath(park),
            linearSlides.goToLowChamber
        ).schedule()
    }
}
package org.firstinspires.ftc.teamcode.opModes.auto

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.pedropathing.localization.Pose
import com.pedropathing.pathgen.PathChain
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import dev.frozenmilk.mercurial.commands.groups.Sequential
import dev.frozenmilk.mercurial.commands.util.Wait
import org.firstinspires.ftc.teamcode.commands.extendoCommand
import org.firstinspires.ftc.teamcode.opModes.MegiddoOpMode
import org.firstinspires.ftc.teamcode.subsystems.clawSubsystem
import org.firstinspires.ftc.teamcode.subsystems.deposit
import org.firstinspires.ftc.teamcode.subsystems.followerSubsystem
import org.firstinspires.ftc.teamcode.subsystems.linearSlides
@Disabled
@Autonomous
class blueBasket : MegiddoOpMode(){
    lateinit var telemetryDB: MultipleTelemetry
    val startingPose = Pose(9.000, 87.882, Math.toRadians(180.0))
    val chamberScore = Pose(36.94117647058824, 84.412, Math.toRadians(180.0))
    val basketScore = Pose(10.059, 128.382, Math.toRadians(310.0-180))
    val sample1 = Pose(22.08823529411765, 118.70588235294117, Math.toRadians(0.0))
//    val sample2 = Pose(20.137, 128.382, Math.toRadians(180.0))
//    val sample3 = Pose(20.137, 131.029, Math.toRadians(200.0))
//    new BezierLine(
//    new Point(9.000, 87.882, Point.CARTESIAN),
//    new Point(43.941, 79.412, Point.CARTESIAN)
//    )
//    )
//    .setConstantHeadingInterpolation(Math.toRadians(180))
    lateinit var scoreFirstSpecimen :PathChain
    lateinit var back :PathChain
    lateinit var intakeSample1 :PathChain


    override fun myInit() {
        deposit.closeClaw()
        deposit.armOut()
//        followerSubsystem.setStartingPose(startingPose)
        linearSlides.runToPosition.cancel()
        telemetryDB = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        scoreFirstSpecimen = followerSubsystem.makeLinePath(startingPose,chamberScore)
        back = followerSubsystem.makeLinePath(chamberScore, startingPose)
        intakeSample1 = followerSubsystem.makeLinePath(chamberScore,sample1)
//        scoreBasket2 = followerSubsystem.makeLinePath(sample1, basketScore)
//        intakeSample2 = followerSubsystem.makeLinePath(basketScore, sample2)
//        scoreBasket3 = followerSubsystem.makeLinePath(sample2, basketScore)
//        intakeSample3 = followerSubsystem.makeLinePath(basketScore, sample3)
//        scoreBasket4 = followerSubsystem.makeLinePath(sample3, basketScore)
//        linearSlides.runToPosition.setFinish{false}
    }

    override fun myStart() {
//        Parallel(
//            followerSubsystem.runFollower,
//            linearSlides.runToPosition.setFinish{false},
//            linearSlides.goToHighChamber,
//            Sequential(
//                followerSubsystem.followPath(scoreFirstSpecimen),
//                Wait(0.2),
//                deposit.slamSeq,
//                Wait(0.2),
//            )
//        ).schedule()
        followerSubsystem.runFollower.schedule()
        linearSlides.runToPosition.schedule()
        Sequential(
            linearSlides.goToHighChamber,
            followerSubsystem.followPath(scoreFirstSpecimen),
            Wait(0.2),
            deposit.slamSeq,
            Wait(0.2),
            extendoCommand.extendoOpenCommand,
            followerSubsystem.followPath(intakeSample1),
            Wait(0.2),
            clawSubsystem.closeClaw,
            Wait(0.3),
            extendoCommand.extendoCloseCommand

        ).schedule()
    }

    override fun myLoop() {
        telemetryDB.addData("slides pose",linearSlides.getPose())
        telemetryDB.addData("slides target",linearSlides.target)
//        followerSubsystem.follower.telemetryDebug(telemetryDB)
        telemetryDB.update()
    }

    override fun myStop() {
//        linearSlides.runToPosition.cancel()
    }
}

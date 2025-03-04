package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.hardware.Servo
import dev.frozenmilk.dairy.cachinghardware.CachingServo
import dev.frozenmilk.dairy.core.FeatureRegistrar
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.util.OpModeLazyCell
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.dairy.pasteurized.Pasteurized
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.commands.groups.Sequential
import dev.frozenmilk.mercurial.commands.util.Wait
import dev.frozenmilk.mercurial.subsystems.SDKSubsystem
import dev.frozenmilk.mercurial.subsystems.Subsystem
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.commands.extendoCommand
import org.firstinspires.ftc.teamcode.util.RunNonBlocking
import org.firstinspires.ftc.teamcode.util.WaitUntil
import org.firstinspires.ftc.teamcode.util.utilCommands
import java.lang.annotation.Inherited
@Config
object deposit : SDKSubsystem() {
    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and
            SingleAnnotation(Mercurial.Attach::class.java)

    @Target(AnnotationTarget.CLASS)
    @Retention(AnnotationRetention.RUNTIME)
    @MustBeDocumented
    @Inherited
    annotation class Attach

    val depoClawServo: CachingServo by OpModeLazyCell {
        val s = CachingServo(
            FeatureRegistrar.activeOpMode.hardwareMap.get(
                Servo::class.java, "small claw"
            )
        )
        s
    }
    val depoArmServo: CachingServo by OpModeLazyCell {
        val s = CachingServo(
            FeatureRegistrar.activeOpMode.hardwareMap.get(
                Servo::class.java, "flip servo"
            )
        )
        s
    }
    val colorSensor: RevColorSensorV3 by OpModeLazyCell {
        val s = FeatureRegistrar.activeOpMode.hardwareMap.get(
            RevColorSensorV3::class.java, "dsc"
        )
        s
    }
    var isSpe = false
    val closeingClawPose = 0.0
    val openingClawPose = 1.0
    @JvmField
    var ArmInPose = 0.23
    val ArmOutPose = 0.71
    val ArmOutPoseParallel = 0.8
    val ArmOutPose2 = 0.9
    @JvmStatic
    fun closeClaw() {
        depoClawServo.setPosition(closeingClawPose)
    }
    @JvmStatic
    fun closeClawRaw() {
        depoClawServo.setPositionRaw(closeingClawPose)
    }

    fun openClaw() {
        depoClawServo.setPosition(openingClawPose)
    }

    fun armIn() {
        depoArmServo.setPosition(ArmInPose)
    }
    @JvmStatic

    fun armOut() {
        depoArmServo.setPosition(ArmOutPose)
    }
    @JvmStatic

    val armOutBasket = Lambda("aob")
        .setInit{
        depoArmServo.setPosition(ArmOutPoseParallel)
    }
    @JvmStatic
    fun armOutHalf() {
        depoArmServo.setPosition(0.5)
    }

    fun armOut2() {
        depoArmServo.setPosition(ArmOutPose2)
    }

    fun transferPose() {
        openClaw()
        armIn()
    }

    fun intakeFromHumanPlayer() {
        openClaw()
        armOut2()
    }

    fun depoPreset() {
        closeClaw()
        armOut()
    }

    fun checkIfSampleInPlace(): Boolean {
        if (Pasteurized.gamepad2.a.onTrue){
            intakeSeq.cancel()
            intakeSeq.schedule()
        }

        if (colorSensor.getDistance(DistanceUnit.MM) < 30) {
            closeClaw()
            return true
        }
        return false
    }

    val slamArm = Lambda("slamArm")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit {
            depoArmServo.position = 0.9
            linearSlides.target -= 400
        }
    @JvmStatic
    val release = Lambda("release")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit { openClaw() }
    val up = Lambda("up")
        .setInit{
            linearSlides.target += 200
        }
    val releaseH = Lambda("Hrelease")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit { depoClawServo.position = 0.42}
    val closeH = Lambda("close")
        .setInit{closeClaw()}
    val quickRC = Sequential(
        WaitUntil{ linearSlides.getPose()>1000},
        releaseH, Wait(0.5), closeH)
    val quickRCSimple = Sequential(releaseH, Wait(0.5), closeH)
    @JvmStatic
    val slamSeq = Sequential(slamArm, Wait(0.2), release,up)
    val changeClawPos = Lambda("changeClawPos")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit {
            if (depoClawServo.position == closeingClawPose) {
                if (isSpe) slamSeq.schedule()
                else openClaw()
            } else {
                closeClaw()
            }
        }

    @JvmStatic
    val armOut = Lambda("armOut")
        .setInit { armOut() }
    @JvmStatic
    val armIn = Lambda("armIn")
        .setInit { armIn() }
    var stop = false
    @JvmStatic
    val intakeCommand = Lambda("intakeCommand")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit {
            intakeFromHumanPlayer()
            linearSlides.runToPosition.schedule()
            stop = false
            extendoCommand.extendoMacro.cancel()
            transferCommand.cancel()
        }
    @JvmStatic
    val catchPixel = Lambda("catchPixel")
        .setFinish {
            checkIfSampleInPlace() || stop
        }
        .setInterruptible{true}
    val catchSimple = Lambda("catchSimple")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit { closeClaw() }
    @JvmStatic
    val postIntakeState = Lambda("postIntakeState")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit { armOut() }
    val intakeSeq = Sequential(
        intakeCommand,
        Wait(0.5),
        catchPixel,
        Wait(0.3),
        postIntakeState
    )


    val transferCommand = Lambda("transferCommand")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit {
            transferPose()
            stop = true

        }
        .setExecute {
            telemetry.addData("active", true)
        }
        .setFinish {
            checkIfSampleInPlace()
        }
    val halfArmIn = Lambda("HAI")
        .setInit{ depoArmServo.position = 0.5}
    val transferSeq = Sequential(
        transferCommand,
//        Wait(0.1),
        RunNonBlocking(intakeSubsystem.outtake.raceWith(Wait(0.5))),
        Wait(0.1),
        halfArmIn
    )
    val transferSeqAuto = Sequential(
        transferCommand.raceWith(Wait(1.1)),
        closeH,
//        Wait(0.1),
        RunNonBlocking(intakeSubsystem.outtake.raceWith(Wait(0.5))),
        Wait(0.1),
        halfArmIn
    )
    val TransferState = Lambda("TransferState")
        .setInit {
            armIn()
            openClaw()
            stop = true
        }


    override fun postUserInitHook(opMode: Wrapper) {
//        armIn()
    }

}
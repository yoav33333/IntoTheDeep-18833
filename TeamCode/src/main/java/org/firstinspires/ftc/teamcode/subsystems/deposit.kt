package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.hardware.ColorRangeSensor
import com.qualcomm.robotcore.hardware.ColorSensor
import com.qualcomm.robotcore.hardware.Servo
import dev.frozenmilk.dairy.cachinghardware.CachingServo
import dev.frozenmilk.dairy.core.FeatureRegistrar
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.util.OpModeLazyCell
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.commands.groups.Sequential
import dev.frozenmilk.mercurial.commands.util.Wait
import dev.frozenmilk.mercurial.subsystems.SDKSubsystem
import dev.frozenmilk.mercurial.subsystems.Subsystem
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.commands.extendoCommand
import org.firstinspires.ftc.teamcode.subsystems.clawSubsystem.clawServo
import org.firstinspires.ftc.teamcode.subsystems.clawSubsystem.closeingPose
import java.lang.annotation.Inherited

object deposit: SDKSubsystem() {
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
    val closeingClawPose = 0.0
    val openingClawPose = 1.0
    val ArmInPose = 0.03
    val ArmOutPose = 0.8

    fun closeClaw() {
        depoClawServo.setPosition(closeingClawPose)
    }

    fun openClaw() {
        depoClawServo.setPosition(openingClawPose)
    }

    fun armIn() {
        depoArmServo.setPosition(ArmInPose)
    }
    fun armOut() {
        depoArmServo.setPosition(ArmOutPose)
    }
    fun transferPose(){
        openClaw()
        armIn()
    }
    fun intakeFromHumanPlayer(){
        openClaw()
        armOut()
    }
    fun depoPreset(){
        closeClaw()
        armOut()
    }

    fun checkIfSampleInPlace(): Boolean {
        if (colorSensor.getDistance(DistanceUnit.MM)<40) {
            closeClaw()
            return true
        }
        return false
    }

    val changeClawPos = Lambda("changeClawPos")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit{
            if (depoClawServo.position == closeingClawPose) {
                openClaw()
            } else {
                closeClaw()
            }
        }

    val release = Lambda("release")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit{ openClaw()}
    val armOut = Lambda("armOut")
        .setInit{armOut()}
    var stop = false
    val intakeCommand = Lambda("intakeCommand")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit{

            intakeFromHumanPlayer()
            stop = false
            extendoCommand.extendoMacro.cancel()
            transferCommand.cancel()
        }
    val catchPixel = Lambda("catchPixel")
        .setFinish{
            checkIfSampleInPlace()|| stop
        }

    val transferCommand = Lambda("transferCommand")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit{
            transferPose()
            stop = true
        }
        .setExecute{
            telemetry.addData("active", true)
        }
        .setFinish{
            checkIfSampleInPlace()
        }

    val transferSeq = Sequential(
        transferCommand,
        Wait(0.1),
        clawSubsystem.openClaw,
        Wait(0.1),
        armOut
    )
    val TransferState = Lambda("TransferState")
        .setInit{
            armIn()
            openClaw()
            stop = true
        }




    override fun postUserInitHook(opMode: Wrapper) {
        closeClaw()
    }

}
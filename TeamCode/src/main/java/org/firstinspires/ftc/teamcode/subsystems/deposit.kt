package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.ColorRangeSensor
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
    val colorSensor: ColorRangeSensor by OpModeLazyCell {
        val s = FeatureRegistrar.activeOpMode.hardwareMap.get(
            ColorRangeSensor::class.java, "dsc"
        )
        s
    }
    val closeingClawPose = 0.0
    val openingClawPose = 1.0
    val ArmInPose = 0.0
    val ArmOutPose = 1.0

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
        return colorSensor.getDistance(DistanceUnit.MM)<40
    }
    val release = Lambda("release")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit{ openClaw()}
    val armOut = Lambda("armOut")
        .setInit{armOut()}
    val transferCommand = Lambda("transferCommand")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit{
            transferPose()
        }
        .setExecute{
            telemetry.addData("active", true)
        }
        .setFinish{
            checkIfSampleInPlace()
        }
        .setEnd{closeClaw()

        }
    val transferSeq = Sequential(
        transferCommand,
        clawSubsystem.openClaw,
        Wait(0.5),
        armOut
        )

    val intakeCommand = Lambda("intakeCommand")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit{
            intakeFromHumanPlayer()
        }
        .setFinish{
            checkIfSampleInPlace()
        }
        .setEnd{closeClaw()}

    override fun postUserInitHook(opMode: Wrapper) {
        closeClaw()
    }

}
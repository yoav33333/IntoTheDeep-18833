package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.ColorSensor
import com.qualcomm.robotcore.hardware.NormalizedColorSensor
import com.qualcomm.robotcore.hardware.Servo
import dev.frozenmilk.dairy.cachinghardware.CachingServo
import dev.frozenmilk.dairy.core.FeatureRegistrar
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.util.OpModeLazyCell
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.subsystems.Subsystem
import org.firstinspires.ftc.robotcontroller.external.samples.SensorColor
import java.lang.annotation.Inherited


object deposit: Subsystem {
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
                Servo::class.java, "dcs"
            )
        )
        s
    }
    val depoArmServo: CachingServo by OpModeLazyCell {
        val s = CachingServo(
            FeatureRegistrar.activeOpMode.hardwareMap.get(
                Servo::class.java, "das"
            )
        )
        s
    }
    val colorSensor: ColorSensor by OpModeLazyCell {
        val s = FeatureRegistrar.activeOpMode.hardwareMap.get(
            ColorSensor::class.java, "sensor_color"
        )
        s
    }
    val closeingClawPose = 1.0
    val openingClawPose = 0.0
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


    override fun postUserInitHook(opMode: Wrapper) {
        closeClaw()
    }

    override fun preUserLoopHook(opMode: Wrapper) {
        armOut()
    }
}
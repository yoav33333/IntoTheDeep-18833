package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.DcMotorEx
import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx
import dev.frozenmilk.dairy.core.FeatureRegistrar
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.util.OpModeLazyCell
import dev.frozenmilk.dairy.core.util.controller.implementation.MotionComponentConsumer
import dev.frozenmilk.dairy.core.util.supplier.numeric.EnhancedDoubleSupplier
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.commands.groups.Advancing
import dev.frozenmilk.mercurial.subsystems.Subsystem
import java.lang.annotation.Inherited


object linearSlides: Subsystem {
    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and
            SingleAnnotation(Mercurial.Attach::class.java)

    @Target(AnnotationTarget.CLASS)
    @Retention(AnnotationRetention.RUNTIME)
    @MustBeDocumented
    @Inherited
    annotation class Attach

    val motorLiftNear: CachingDcMotorEx by OpModeLazyCell {
        val s = CachingDcMotorEx(
            FeatureRegistrar.activeOpMode.hardwareMap.get(
                DcMotorEx::class.java, "mln"
            )
        )
        s
    }
    val motorLiftMiddle: CachingDcMotorEx by OpModeLazyCell {
        val s = CachingDcMotorEx(
            FeatureRegistrar.activeOpMode.hardwareMap.get(
                DcMotorEx::class.java, "mlm"
            )
        )
        s
    }
    val motorLiftFar: CachingDcMotorEx by OpModeLazyCell {
        val s = CachingDcMotorEx(
            FeatureRegistrar.activeOpMode.hardwareMap.get(
                DcMotorEx::class.java, "mlf"
            )
        )
        s
    }
    val closeingPose = 0.0


    fun setMotorPower(power: EnhancedDoubleSupplier) {
        motorLiftNear.power = power.state
        motorLiftMiddle.power = power.state
        motorLiftFar.power = power.state
    }
    fun motorPowerSupplier(): EnhancedDoubleSupplier {
        return EnhancedDoubleSupplier{motorLiftNear.power}
    }
    fun getMotorPower(): Double {
        return motorLiftNear.power

    }
    fun getMotorPos(): EnhancedDoubleSupplier {
        return EnhancedDoubleSupplier{motorLiftNear.currentPosition.toDouble()}

    }
    var holdingPose = 0.0
    var PID_Target = EnhancedDoubleSupplier{holdingPose}
    val p = 0.0
    val i = 0.0
    val d = 0.0
    val controller = PID_Controller(getMotorPos(), PID_Target,  p, i, d)
        .doubleController

    val PID_Power = EnhancedDoubleSupplier{holdingPose}

    fun holdPose() {
        runToPose(getMotorPos().state)
    }
    fun runToPose(pose: Double) {
        controller.enabled = true
        PID_Target.state = holdingPose
        setMotorPower(PID_Power)
    }

    fun closeSlides(){
        runToPose(closeingPose)
    }
    fun runManually(power: Double){
        controller.enabled = false
        setMotorPower(EnhancedDoubleSupplier{ power })
    }
    val manualControl = Lambda("manualControl")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit{
            runManually(Mercurial.gamepad2.rightStickY.state)}
    val holdPose = Lambda("holdSlidesPose")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit{
            holdPose()
        }
    val closeSlides = Lambda("closeSlides")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit{closeSlides()}

    override fun preUserInitHook(opMode: Wrapper) {
        controller.outputConsumer = MotionComponentConsumer { PID_Power}
    }



}
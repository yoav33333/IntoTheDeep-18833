package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DigitalChannel
import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx
import dev.frozenmilk.dairy.core.FeatureRegistrar
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.util.OpModeLazyCell
import dev.frozenmilk.dairy.core.util.supplier.logical.EnhancedBooleanSupplier
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.bindings.BoundBooleanSupplier
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.commands.groups.Sequential
import dev.frozenmilk.mercurial.subsystems.Subsystem
import org.firstinspires.ftc.teamcode.controller.PDController
import org.firstinspires.ftc.teamcode.subsystems.deposit.armOut
import org.firstinspires.ftc.teamcode.subsystems.deposit.isSpe
import org.firstinspires.ftc.teamcode.subsystems.deposit.quickRC
import org.firstinspires.ftc.teamcode.util.utilCommands
import java.lang.annotation.Inherited
import kotlin.math.abs

@Config
object linearSlides : Subsystem {
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
                DcMotorEx::class.java, "l1"
            )
        )
        s.cachingTolerance = 0.01
        s
    }
    val motorLiftMiddle: CachingDcMotorEx by OpModeLazyCell {
        val s = CachingDcMotorEx(
            FeatureRegistrar.activeOpMode.hardwareMap.get(
                DcMotorEx::class.java, "l2"
            )
        )
        s.cachingTolerance = 0.01
        s
    }
    val motorLiftFar: CachingDcMotorEx by OpModeLazyCell {
        val s = CachingDcMotorEx(
            FeatureRegistrar.activeOpMode.hardwareMap.get(
                DcMotorEx::class.java, "l3"
            )
        )
        s.cachingTolerance = 0.01
        s
    }

    val magneticLimit: DigitalChannel by OpModeLazyCell {
        val s = FeatureRegistrar.activeOpMode.hardwareMap.get(
            DigitalChannel::class.java, "magneticLimit"
        )
        s.mode = DigitalChannel.Mode.INPUT
        s
    }

    @JvmField
    var target = 0.0

    @JvmField
    var Kp = 0.0045

    @JvmField
    var Kd = 0.001

    @JvmField
    var Kf = 0.02

    @JvmField
    var maxPow = 1.0

    @JvmField
    var threshold = 30.0

    var PDController = PDController(Kp, Kd)
    val closeingPose = 0.0


    fun runToPose(pose: Double) {
        PDController = PDController(Kp, Kd)
        setPower(PDController.calculate(getPose().toDouble(), pose))
    }

    fun closeSlides() {
        runToPose(closeingPose)
    }

    fun setPower(power: Double) {
        motorLiftNear.setPower(power)
        motorLiftMiddle.setPower(power)
        motorLiftFar.setPower(power)
    }

    fun setZeroPowerBehavior(zeroPowerBehavior: DcMotor.ZeroPowerBehavior) {
        motorLiftNear.zeroPowerBehavior = zeroPowerBehavior
        motorLiftMiddle.zeroPowerBehavior = zeroPowerBehavior
        motorLiftFar.zeroPowerBehavior = zeroPowerBehavior
    }

    fun setRunMode(mode: RunMode) {
        motorLiftNear.mode = mode
        motorLiftMiddle.mode = mode
        motorLiftFar.mode = mode
    }

    var offset = 0
    @JvmStatic
    fun getPose(): Int {
        return motorLiftNear.currentPosition + offset
    }
    @JvmStatic
    fun setPose(pose: Int) {
        offset = pose
    }

    val resetHeight = Lambda("resetHeight")
        .setExecute { offset -= (getPose()) }
        .setFinish{false}
    val manualControl = Lambda("manualControl")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit { runToPosition.cancel() }
        .setExecute {runToPosition.cancel()
            setPower(Mercurial.gamepad2.rightStickY.state) }
        .setFinish { abs(Mercurial.gamepad2.rightStickY.state) < 0.1 }
        .setEnd { target = getPose().toDouble() }

    @JvmStatic
    val runToPosition = Lambda("runToPosition")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setExecute {
            runToPose(target)
        }
        .setFinish { abs(Mercurial.gamepad2.rightStickY.state) > 0.1 || !FeatureRegistrar.opModeRunning}
    val nonBlockRTP = Lambda("nonBlockRTP")
        .setInit{ runToPosition.schedule()}
    fun goToPreset(goal: Double) = Lambda("goToPreset")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit { target = goal }

    val stopRunToPosition = Lambda("stopRunToPosition")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit { runToPosition.cancel() }

    val closeSlides =
        goToPreset(0.0).setFinish { abs(getPose()) < 40 }.setEnd { runToPosition.cancel() }
            .addInit {
                runToPosition.schedule()
                target = 0.0
                isSpe = false
            }
    val closeSlidesAuto =
        goToPreset(0.0).setFinish { abs(getPose()) < 60 }
            .addInit {
                runToPosition.schedule()
                target = 0.0
                isSpe = false
            }
            .setEnd { runToPosition.cancel() }
    @JvmStatic
    val goToHighBasket = goToPreset(3500.0).addInit { isSpe = false
//        Sequential(
//            utilCommands.waitUntil{ getPose()>3300 || abs(Mercurial.gamepad2.rightStickY.state)>0.2 },
//            armOut
//        ).schedule()
    }
    val goToLowBasket = goToPreset(1700.0).addInit { isSpe = false
//        Sequential(
//            utilCommands.waitUntil{ getPose()>1500 || abs(Mercurial.gamepad2.rightStickY.state)>0.2 },
//            armOut
//        ).schedule()
    }
    @JvmStatic
    val goToHighChamber = goToPreset(1160.0).addInit { isSpe = true
        quickRC.schedule()
//        Sequential(
//            utilCommands.waitUntil{ getPose()>500 || abs(Mercurial.gamepad2.rightStickY.state)>0.2 },
//            armOut
//        ).schedule()
    }
    @JvmStatic
    val touchBar = goToPreset(1400.0).addInit { isSpe = true
        quickRC.schedule()}
    @JvmStatic
    val goToLowChamber = goToPreset(0.0).addInit { isSpe = true
        quickRC.schedule()
//        Sequential(
//            utilCommands.waitUntil{ Mercurial.isScheduled(runToPosition) || abs(Mercurial.gamepad2.rightStickY.state)>0.2 },
//            armOut
//        ).schedule()
    }
    @JvmStatic
    val goToLowChamberNoRC = goToPreset(0.0).addInit { isSpe = true }

    override fun preUserInitHook(opMode: Wrapper) {
        setRunMode(RunMode.STOP_AND_RESET_ENCODER)
        setRunMode(RunMode.RUN_WITHOUT_ENCODER)
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        BoundBooleanSupplier(EnhancedBooleanSupplier { !magneticLimit.state })
            .whileTrue(resetHeight)
    }


}
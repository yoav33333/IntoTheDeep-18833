package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
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
import dev.frozenmilk.mercurial.subsystems.Subsystem
import org.firstinspires.ftc.teamcode.controller.PDController
import org.firstinspires.ftc.teamcode.subsystems.deposit.isSpe
import org.firstinspires.ftc.teamcode.subsystems.deposit.quickRC
import org.firstinspires.ftc.teamcode.util.Encoder
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

    val leftCenter: CachingDcMotorEx by OpModeLazyCell {
        val s = CachingDcMotorEx(
            FeatureRegistrar.activeOpMode.hardwareMap.get(
                DcMotorEx::class.java, "l1"
            )
        )
        s.cachingTolerance = 0.01
        s
    }
    val leftSide: CachingDcMotorEx by OpModeLazyCell {
        val s = CachingDcMotorEx(
            FeatureRegistrar.activeOpMode.hardwareMap.get(
                DcMotorEx::class.java, "l2"
            )
        )
        s.cachingTolerance = 0.01
        s
    }
    val rightCenter: CachingDcMotorEx by OpModeLazyCell {
        val s = CachingDcMotorEx(
            FeatureRegistrar.activeOpMode.hardwareMap.get(
                DcMotorEx::class.java, "l3"
            )
        )
        s.cachingTolerance = 0.01
        s.direction = DcMotorSimple.Direction.REVERSE
        s
    }
    val rightSide: CachingDcMotorEx by OpModeLazyCell {
        val s = CachingDcMotorEx(
            FeatureRegistrar.activeOpMode.hardwareMap.get(
                DcMotorEx::class.java, "l4"
            )
        )
        s.cachingTolerance = 0.01
        s.direction = DcMotorSimple.Direction.REVERSE
        s
    }
    val leftEncoder: Encoder by OpModeLazyCell{
        Encoder("l1")
    }
    val rightEncoder: Encoder by OpModeLazyCell{
        Encoder("l4", DcMotorSimple.Direction.REVERSE)
    }

    val magneticLimit: DigitalChannel by OpModeLazyCell {
        val s = FeatureRegistrar.activeOpMode.hardwareMap.get(
            DigitalChannel::class.java, "magneticLimit"
        )
        s.mode = DigitalChannel.Mode.INPUT
        s
    }
    @JvmStatic
    var startingPose = 0
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
        setPower(PDController.calculate(getPoseRight().toDouble(), pose), PDController.calculate(getPoseRight().toDouble(), pose))
    }

    fun closeSlides() {
        runToPose(closeingPose)
    }

    fun setPower(powerRight: Double, powerLeft: Double? = Double.NaN) {
        (powerLeft ?: powerRight).let { leftCenter.setPower(it) }
        (powerLeft ?: powerRight).let { leftSide.setPower(it) }
        rightSide.setPower(powerRight)
        rightSide.setPower(powerRight)
    }

    fun setZeroPowerBehavior(zeroPowerBehavior: DcMotor.ZeroPowerBehavior) {
        leftCenter.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        leftSide.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        rightSide.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        rightSide.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }

    fun setRunMode(mode: RunMode) {
        leftCenter.mode = mode
        leftSide.mode = mode
        rightSide.mode = mode
        rightSide.mode = mode
    }

    var offset = 0
    fun getPoseRight() = rightEncoder.getPose()
    fun getPoseLeft() = leftEncoder.getPose()
    @JvmStatic
    fun getPose(): Int {
        return (getPoseLeft()+ getPoseRight()/2)+ offset
    }
    @JvmStatic
    fun setPose(pose: Int) {
        offset -= getPose() - pose
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
//    val justCloseSlides =
//        goToPreset(0.0).setFinish { abs(getPose()) < 40 }.setEnd { runToPosition.cancel() }
//            .addInit {
//                runToPosition.schedule()
//                target = 0.0
//                isSpe = false
//            }
    var lastPose = 0
    fun getVel():Int{
        var temp = lastPose
        lastPose = getPose()
        return temp- getPose()
    }
    val closeSlidesDumb = Lambda("closeSlidesDumb")
        .setInit{
            runToPosition.cancel()

            lastPose = 0
        }
        .setExecute{setPower(-1.0)}
        .setFinish{ !magneticLimit.state}
        .setEnd{ setPower(0.0)}

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
    val touchBar = goToPreset(1480.0).addInit { isSpe = true
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
        setPose(startingPose)

    }


}
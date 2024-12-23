package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotorEx
import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx
import dev.frozenmilk.dairy.core.FeatureRegistrar
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.util.OpModeLazyCell
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.subsystems.Subsystem
import org.firstinspires.ftc.teamcode.controller.PDController
import org.firstinspires.ftc.teamcode.util.motorGroup
import java.lang.annotation.Inherited
import kotlin.math.abs


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
    var Kp = 1.0
    var Kd = 1.0
//    val PDController = PDController(Kp, Kd)
//    val motors = motorGroup(motorLiftNear, motorLiftMiddle, motorLiftFar)
    val closeingPose = 0.0
    var target = 100.0

//    val p = 0.0
//    val i = 0.0
//    val d = 0.0

//    val doubleController = DoubleController(
//        // target
//        // NaN can be returned for a component if you want to completely ignore it
//        // but usually something else is better: 0.0, NEGATIVE_INFINITY, POSITIVE_INFINITY
//        // in this case we're only ever going to use state for a calculation
//        targetSupplier = MotionComponentSupplier {
//            if (it == MotionComponents.STATE) {
//                return@MotionComponentSupplier target
//            }
//            0.0
//        },
//        // state
//        // we'll use the motor's encoder for feedback
//        stateSupplier = { motorLiftMiddle.currentPosition.toDouble() },
//        // tolerance
//        // when we check if we're finished, this is our default allowable error
//        // NaN can be returned for a component if you want to completely ignore it
//        // this cached wrapper will prevent regenerating the outputs, as they aren't dynamic
//        toleranceEpsilon = CachedMotionComponentSupplier(
//            MotionComponentSupplier {
//                return@MotionComponentSupplier when (it) {
//                    MotionComponents.STATE -> 10.0
//                    MotionComponents.VELOCITY -> 1.0
//                    else -> Double.NaN
//                }
//            }
//        ),
//        // optional, callback
//        outputConsumer = motors::setPower, // when this controller updates, this callback will be run
//        // then we build up the calculation:
//        controllerCalculation = DoubleComponent.P(MotionComponents.STATE, 0.1) // first P
//            .plus(DoubleComponent.I(MotionComponents.STATE, -0.00003, -0.1, 0.1)) // then I
//            .plus(DoubleComponent.D(MotionComponents.STATE, 0.0005)) // then D
//    )

//    fun holdPose() {
//        runToPose(motors.getPos().toDouble())
//    }
//    fun runToPose(pose: Double) {
//        motors.setPower(PDController.calculate(motors.getPos().toDouble(), pose))
//        target = pose
//
//    }
//
//    fun closeSlides(){
//        runToPose(closeingPose)
//    }
    fun runManually(power: Double){
        motorLiftNear.setPower( power )
        motorLiftMiddle.setPower( power )
        motorLiftFar.setPower( power )
    }
    val manualControl = Lambda("manualControl")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit{runManually(Mercurial.gamepad2.rightStickY.state)}
        .setExecute{runManually(Mercurial.gamepad2.rightStickY.state)}
        .setFinish{false}
//
//    val holdPose = Lambda("holdSlidesPose")
//        .setRunStates(Wrapper.OpModeState.ACTIVE)
//        .setExecute{
//            holdPose()
//        }
//        .setFinish{false}
//    val closeSlides = Lambda("closeSlides")
//        .setRunStates(Wrapper.OpModeState.ACTIVE)
//        .setInit{closeSlides()}
//        .setExecute{closeSlides()}
//        .setFinish{ abs(target- motors.getPos())<20}

//    override fun preUserInitHook(opMode: Wrapper) {
//
//        motors.setRunMode(RunMode.STOP_AND_RESET_ENCODER)
//        motors.setRunMode(RunMode.RUN_WITHOUT_ENCODER)
//        motors.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
//    }
//    val telemetry = MultipleTelemetry(FeatureRegistrar.activeOpMode.telemetry, FtcDashboard.getInstance().telemetry)


//    override fun postUserLoopHook(opMode: Wrapper) {
//        telemetry.addData("pose", motors.getPos())
//        telemetry.addData("target", target)
//        telemetry.addData("error", target - motors.getPos())
//        telemetry.update()
//
//    }
}
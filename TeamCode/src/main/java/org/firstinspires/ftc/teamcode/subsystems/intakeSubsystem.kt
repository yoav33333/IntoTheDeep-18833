package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.ColorRangeSensor
import com.qualcomm.robotcore.hardware.Servo
import dev.frozenmilk.dairy.core.FeatureRegistrar
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.util.OpModeLazyCell
import dev.frozenmilk.dairy.core.util.supplier.numeric.unit.EnhancedUnitSupplier
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.commands.groups.Sequential
import dev.frozenmilk.mercurial.commands.util.Wait
import dev.frozenmilk.mercurial.subsystems.SDKSubsystem
import dev.frozenmilk.mercurial.subsystems.Subsystem
import org.firstinspires.ftc.teamcode.util.InstantCommand
import java.lang.annotation.Inherited
import java.util.function.Supplier

@Config
object intakeSubsystem : SDKSubsystem() {
    enum class Color{
        RED,
        BLUE,
        YELLOW
    }
    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and
            SingleAnnotation(Mercurial.Attach::class.java)

    @Target(AnnotationTarget.CLASS)
    @Retention(AnnotationRetention.RUNTIME)
    @MustBeDocumented
    @Inherited
    annotation class Attach

    val leftWheel: Servo by OpModeLazyCell {
        val s =
            FeatureRegistrar.activeOpMode.hardwareMap.get(
                Servo::class.java, "leftWheel"

            )
        s
    }
    val colorDistSensor: ColorRangeSensor by OpModeLazyCell {
        FeatureRegistrar.activeOpMode.hardwareMap.get(
            ColorRangeSensor::class.java, "color claw"
        )
    }
    val rightWheel: Servo by OpModeLazyCell {
        val s =
            FeatureRegistrar.activeOpMode.hardwareMap.get(
                Servo::class.java, "rightWheel"

            )
        s.direction = Servo.Direction.REVERSE
        s
    }
    val openCloseServo: Servo by OpModeLazyCell {
        val s =
            FeatureRegistrar.activeOpMode.hardwareMap.get(
                Servo::class.java, "openCloseServo"

            )
        s.direction = Servo.Direction.REVERSE
        s
    }
    @JvmField
    var redThreshold = 0.068
    @JvmField
    var blueThreshold = 0.016
    fun getColor():Color{
        var colors = colorDistSensor.normalizedColors
        telemetry.addData("red", colors.red)
        telemetry.addData("blue", colors.blue)
//        telemetry.addData("red", colors.red)
        if (colors.red > redThreshold)
            return Color.YELLOW
        if (colors.blue > blueThreshold)
            return Color.BLUE
        return Color.YELLOW
    }

    @JvmField
    var open =0.58
    @JvmField
    var close = 0.51
    val outtake = Lambda("outtake")
        .setExecute{
            rightWheel.position = 0.0
            leftWheel.position = 1.0
        }
        .setFinish{false}
        .setEnd{ rightWheel.position = 0.5
            leftWheel.position = 0.5}

    val ejectSeq = Sequential(armClawSubsystem.ejectAngle, InstantCommand{ ejecting = true}, outtake.raceWith(Wait(0.5)), armClawSubsystem.openClawArm, InstantCommand{ ejecting = false})
    var ejecting = false
    val intake = Lambda("intake")
        .setExecute{
            if (!ejecting){
                rightWheel.position = 1.0
                leftWheel.position = 0.0
//                ejectSeq.schedule()
            }
//            else {
//
//            }
        }
        .setFinish{false}
        .setEnd{ rightWheel.position = 0.5
            leftWheel.position = 0.5}

    val openIntake = Lambda("oi")
        .setInit{ openCloseServo.position = open}
        .setFinish{true}
    val closeIntake = Lambda("ci")
        .setInit{ openCloseServo.position = close}
        .setFinish{true}
    val closeIntakeFull = Lambda("cif")
        .setInit{ openCloseServo.position = close-0.1}
        .setFinish{true}
    val changeClawPos = Lambda("ccp")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit{
            if (openCloseServo.position == open){
                openCloseServo.position = close
            }
            else{
                openCloseServo.position = open
            }
        }
        .setFinish{true}

    override fun postUserLoopHook(opMode: Wrapper) {
        telemetry.addData("color", getColor())
    }
}

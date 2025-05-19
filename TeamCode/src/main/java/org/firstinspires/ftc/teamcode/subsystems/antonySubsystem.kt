package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.hardware.rev.RevBlinkinLedDriver
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
import dev.frozenmilk.mercurial.commands.util.Wait
import dev.frozenmilk.mercurial.subsystems.SDKSubsystem
import dev.frozenmilk.mercurial.subsystems.Subsystem
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit
import org.firstinspires.ftc.teamcode.subsystems.BulkReads.modules
import java.lang.annotation.Inherited


object antonySubsystem : SDKSubsystem() {
    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and
            SingleAnnotation(Mercurial.Attach::class.java)

    @Target(AnnotationTarget.CLASS)
    @Retention(AnnotationRetention.RUNTIME)
    @MustBeDocumented
    @Inherited
    annotation class Attach

    val antony: RevBlinkinLedDriver by OpModeLazyCell {
        val s =
            FeatureRegistrar.activeOpMode.hardwareMap.get(
                RevBlinkinLedDriver::class.java, "LED"
            )
        s
    }
    val default = RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_FOREST_PALETTE
    val endGame = RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_RAINBOW_PALETTE
    val lowBattery = RevBlinkinLedDriver.BlinkinPattern.RED
    val redLED = RevBlinkinLedDriver.BlinkinPattern.RED
    val yellowLED = RevBlinkinLedDriver.BlinkinPattern.YELLOW
    val blueLED = RevBlinkinLedDriver.BlinkinPattern.BLUE
    val greenLED = RevBlinkinLedDriver.BlinkinPattern.GREEN

    val confirmation = Lambda("confirmation")
        .setExecute{
//            if (clawSubsystem.colorDistSensor.getDistance(DistanceUnit.MM)<30){
//                antony.setPattern(greenLED)
//            }
//            else{
//                antony.setPattern(redLED)
//            }
        }
        .setFinish{false}
    val endGameCommand = Lambda("endGameCommand")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit { antony.setPattern(endGame) }
        .setFinish { false }
        .setEnd { antony.setPattern(default) }

    val lowBatteryCommand = Lambda("lowBatteryCommand")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setExecute { antony.setPattern(lowBattery) }
        .setFinish{false}
        .setEnd { antony.setPattern(default) }

    var data = 0
    var red = 0
    var blue = 0
    var green = 0
    fun csColors() {
//        data =  clawSubsystem.colorDistSensor.argb()
        red = clawSubsystem.colorDistSensor.red()
        blue = clawSubsystem.colorDistSensor.blue()
        green = clawSubsystem.colorDistSensor.green()

        telemetry.update()
        if (blue > 300) {
            antony.setPattern(blueLED)
        } else if (green > 300) {
            antony.setPattern(yellowLED)
        } else if (red > 290) {
            antony.setPattern(redLED)
        } else antony.setPattern(default)
    }

    val colorSensorData = Lambda("colorSensorData")
        .setExecute {
            csColors()
        }
        .setFinish { false }
        .setEnd { antony.setPattern(default) }

    val minVoltage = 10.0


    override fun postUserStartHook(opMode: Wrapper) {
        antony.setPattern(default)
//        BoundBooleanSupplier(EnhancedBooleanSupplier {
//            FeatureRegistrar.activeOpMode.runtime > 80 || FeatureRegistrar.activeOpMode.runtime > 110
//        })
//            .onTrue(endGameCommand.raceWith(Wait(10.0)))
        Sequential(
            Wait(80.0),
            endGameCommand.raceWith(Wait(10.0)),
            Wait(20.0),
            endGameCommand.raceWith(Wait(10.0))
        )

        BoundBooleanSupplier(EnhancedBooleanSupplier
        { if (modules.isEmpty()) false else modules[0].getInputVoltage(VoltageUnit.VOLTS) < 9.0 })
            .whileTrue(lowBatteryCommand)
    }

}
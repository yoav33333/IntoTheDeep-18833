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
import dev.frozenmilk.mercurial.commands.util.Wait
import dev.frozenmilk.mercurial.subsystems.Subsystem
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit
import org.firstinspires.ftc.teamcode.subsystems.BulkReads.modules
import java.lang.annotation.Inherited

object antonySubsystem : Subsystem {
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
    val default = RevBlinkinLedDriver.BlinkinPattern.TWINKLES_FOREST_PALETTE
    val endGame = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_PARTY_PALETTE
    val lowBattery = RevBlinkinLedDriver.BlinkinPattern.RED

    val endGameCommand = Lambda("endGameCommand")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit{antony.setPattern(endGame)}
        .setFinish{false}
        .setEnd{antony.setPattern(default)}

    val lowBatteryCommand = Lambda("lowBatteryCommand")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setExecute{antony.setPattern(lowBattery)}
        .setEnd{antony.setPattern(default)}

    val minVoltage = 10.0


    override fun postUserInitHook(opMode: Wrapper) {
        BoundBooleanSupplier(EnhancedBooleanSupplier{
            FeatureRegistrar.activeOpMode.runtime>80||FeatureRegistrar.activeOpMode.runtime>110})
            .onTrue(endGameCommand.raceWith(Wait(10.0)))

        BoundBooleanSupplier(EnhancedBooleanSupplier
        {if (modules.isEmpty()) false else modules[0].getInputVoltage(VoltageUnit.VOLTS)<9.0})
            .whileTrue(lowBatteryCommand)
    }

}
package org.firstinspires.ftc.teamcode.subsystems.antony

import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import dev.frozenmilk.dairy.core.FeatureRegistrar
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.util.OpModeLazyCell
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.bindings.BoundBooleanSupplier
import dev.frozenmilk.mercurial.commands.groups.Sequential
import dev.frozenmilk.mercurial.commands.util.Wait
import dev.frozenmilk.mercurial.subsystems.SDKSubsystem
import dev.frozenmilk.mercurial.subsystems.Subsystem
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit
import org.firstinspires.ftc.teamcode.subsystems.robot.BulkReads.modules
import org.firstinspires.ftc.teamcode.subsystems.antony.AntonyCommands.setEndGame
import org.firstinspires.ftc.teamcode.subsystems.antony.AntonyCommands.setLowBattery
import java.lang.annotation.Inherited
import dev.frozenmilk.dairy.core.util.supplier.logical.EnhancedBooleanSupplier
import dev.frozenmilk.mercurial.commands.Command
import org.firstinspires.ftc.teamcode.subsystems.antony.AntonyCommands.setDefault
import org.firstinspires.ftc.teamcode.subsystems.antony.AntonyVariables.minVoltThreshold
import org.firstinspires.ftc.teamcode.util.HardwareDevice

object AntonyHardware: SDKSubsystem() {
    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and
            SingleAnnotation(Mercurial.Attach::class.java)

    @Target(AnnotationTarget.CLASS)
    @Retention(AnnotationRetention.RUNTIME)
    @MustBeDocumented
    @Inherited
    annotation class Attach

    val antony by OpModeLazyCell{ HardwareDevice("LED", RevBlinkinLedDriver::class.java).get() }

    fun setPattern(pattern: RevBlinkinLedDriver.BlinkinPattern) {
        antony.setPattern(pattern)
    }

    override var defaultCommand: Command? = setDefault

    override fun postUserStartHook(opMode: Wrapper) {
        Sequential(
            Wait(80.0),
            setEndGame.raceWith(Wait(10.0)),
            Wait(20.0),
            setEndGame.raceWith(Wait(10.0))
        ).schedule()

        BoundBooleanSupplier(EnhancedBooleanSupplier
        { if (modules.isEmpty()) false else modules[0].getInputVoltage(VoltageUnit.VOLTS) < minVoltThreshold })
            .whileTrue(setLowBattery)
    }
}
package org.firstinspires.ftc.teamcode.subsystems.drive

import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.commands.Lambda
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveHardware.drive
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveHardware.setIMUHeading
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveVariables.maxPower
import java.util.function.DoubleSupplier

object DriveCommands {
    fun driveCommand(x: DoubleSupplier, y: DoubleSupplier, rotation: DoubleSupplier, robotCentric: () -> Boolean = { false })
        =Lambda("DriveCommand")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit{maxPower = 1.0}
        .setExecute{drive(x.asDouble, y.asDouble, rotation.asDouble, robotCentric())}

    fun setHeading(heading: Double) = Lambda("setHeading")
        .setRunStates(Wrapper.OpModeState.ACTIVE, Wrapper.OpModeState.INIT)
        .setInit { setIMUHeading(heading) }

    val resetHeading = setHeading(0.0)

}
package org.firstinspires.ftc.teamcode.subsystems.drive

import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.dairy.pasteurized.Pasteurized.gamepad1
import dev.frozenmilk.dairy.pasteurized.Pasteurized.gamepad2
import dev.frozenmilk.mercurial.commands.Lambda
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveHardware.drive
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveHardware.setIMUHeading
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveVariables.highGear
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveVariables.lowPGear
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveVariables.maxPower
import java.util.function.BooleanSupplier
import java.util.function.DoubleSupplier

object DriveCommands {
    fun driveCommand(x: DoubleSupplier, y: DoubleSupplier, rotation: DoubleSupplier, robotCentric: BooleanSupplier)
        =Lambda("DriveCommand")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit{maxPower = 1.0}
        .setExecute{
            drive(
            gamepad1.leftTrigger.state + gamepad1.rightTrigger.state>0.1
        )}
        .setFinish{false}

    fun setHeading(heading: Double) = Lambda("setHeading")
        .setRunStates(Wrapper.OpModeState.ACTIVE, Wrapper.OpModeState.INIT)
        .setInit { setIMUHeading(heading) }

    val resetHeading = setHeading(0.0)

    val runLowGear = Lambda("lowGear")
        .setInit{ maxPower = lowPGear}
        .setEnd{ maxPower = highGear}
        .setFinish{false}

}
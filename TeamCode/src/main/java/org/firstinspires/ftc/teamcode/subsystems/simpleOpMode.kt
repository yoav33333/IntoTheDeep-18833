package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.frozenmilk.dairy.core.util.supplier.logical.EnhancedBooleanSupplier
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.bindings.BoundBooleanSupplier
import dev.frozenmilk.mercurial.bindings.BoundDoubleSupplier
import dev.frozenmilk.mercurial.commands.util.IfElse
import dev.frozenmilk.util.cell.RefCell
import org.firstinspires.ftc.teamcode.commands.extendoCommand
import kotlin.math.abs

@Mercurial.Attach
@clawSubsystem.Attach
@driveSubsystem.Attach
@extendoCommand.Attach
@linearSlides.Attach
@BulkReads.Attach
@TeleOp
class simpleOpMode : OpMode() {

    override fun init() {


        Mercurial.gamepad2.y.onTrue(
            clawSubsystem.changeClawPos
        )


        BoundBooleanSupplier(EnhancedBooleanSupplier{
            abs(Mercurial.gamepad2.leftStickX.state)>0.1})
            .whileTrue(clawSubsystem.rotateClaw)

        BoundBooleanSupplier(EnhancedBooleanSupplier{
            abs(Mercurial.gamepad2.rightStickY.state)>0.1})
            .onTrue(linearSlides.manualControl)
            .onFalse(linearSlides.holdPose)

        Mercurial.gamepad2.a.onTrue(
            linearSlides.closeSlides
        )

        Mercurial.gamepad2.x.onTrue(
            extendoCommand.changeExtendoState
        )


    }



override fun loop() {
    telemetry.addData("", clawSubsystem.leftClawServo.position)
    telemetry.addData("", clawSubsystem.rightClawServo.position)
    telemetry.addData("", clawSubsystem.clawRotationServo.position)
    telemetry.addData("", extendoCommand.currentExtendoState)

    telemetry.update()

    }

}
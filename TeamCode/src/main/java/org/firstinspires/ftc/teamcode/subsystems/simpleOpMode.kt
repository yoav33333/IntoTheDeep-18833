package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.frozenmilk.dairy.core.util.supplier.logical.EnhancedBooleanSupplier
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.bindings.BoundBooleanSupplier
import dev.frozenmilk.mercurial.bindings.BoundDoubleSupplier
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.commands.groups.Parallel
import dev.frozenmilk.mercurial.commands.util.StateMachine
import dev.frozenmilk.util.cell.RefCell
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry
import kotlin.math.abs

@Mercurial.Attach
@clawSubsystem.Attach
@driveSubsystem.Attach
@TeleOp
class simpleOpMode : OpMode() {

    enum class extendoState{
        OPEN,
        CLOSE;
    }

    override fun init() {
        val currentExtendoState = extendoState.CLOSE

        Mercurial.gamepad2.y.onTrue(
            clawSubsystem.changeClawPos
        )
        BoundBooleanSupplier(EnhancedBooleanSupplier{
            abs(Mercurial.gamepad2.leftStickX.state)>0.1 })
            .whileTrue(clawSubsystem.rotateClaw)

        Mercurial.gamepad2.x.onTrue(StateMachine(extendoState.CLOSE)
            .withState(extendoState.CLOSE){state: RefCell<extendoState>, name: String ->
                Parallel(
                    armClawSubsystem.openClawArm,
                    extendoSubsystem.openExtendo
                        .addEnd{ state.accept(extendoState.OPEN)},
                )
            }
            .withState(extendoState.OPEN){state: RefCell<extendoState>, name: String ->
                Parallel(
                    armClawSubsystem.closeClawArm,
                    extendoSubsystem.closeExtendo
                        .addEnd{ state.accept(extendoState.CLOSE)},
                )
            }
        )
    }



override fun loop() {
        telemetry.addData("", clawSubsystem.leftClawServo.position)
        telemetry.addData("", clawSubsystem.rightClawServo.position)
        telemetry.addData("", clawSubsystem.clawRotationServo.position)
        telemetry.update()

    }

}
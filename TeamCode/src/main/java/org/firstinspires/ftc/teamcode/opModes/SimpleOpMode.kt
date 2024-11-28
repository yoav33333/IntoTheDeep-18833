package org.firstinspires.ftc.teamcode.opModes

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.frozenmilk.dairy.core.util.supplier.logical.EnhancedBooleanSupplier
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.bindings.BoundBooleanSupplier
import dev.frozenmilk.mercurial.bindings.BoundDoubleSupplier
import dev.frozenmilk.mercurial.commands.util.StateMachine
import dev.frozenmilk.mercurial.commands.util.Wait
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.commands.extendoCommand
import org.firstinspires.ftc.teamcode.commands.extendoCommand.extendoCloseCommand
import org.firstinspires.ftc.teamcode.commands.extendoCommand.extendoOpenCommand
import org.firstinspires.ftc.teamcode.subsystems.BulkReads
import org.firstinspires.ftc.teamcode.subsystems.armClawSubsystem
import org.firstinspires.ftc.teamcode.subsystems.clawSubsystem
import org.firstinspires.ftc.teamcode.subsystems.driveSubsystem
import org.firstinspires.ftc.teamcode.subsystems.driveSubsystem.robotOrientedDrive
import org.firstinspires.ftc.teamcode.subsystems.extendoSubsystem
import kotlin.math.abs

@Mercurial.Attach
@clawSubsystem.Attach
@driveSubsystem.Attach
@extendoCommand.Attach
//@linearSlides.Attach
@BulkReads.Attach
@extendoSubsystem.Attach
@TeleOp
class simpleOpMode : OpMode() {

    lateinit var ec : StateMachine<Boolean>
    override fun init() {


        Mercurial.gamepad2.y.onTrue(
            clawSubsystem.changeClawPos
        )
        Mercurial.gamepad2.leftStickButton.onTrue(
            clawSubsystem.resetAngleClaw
        )

        BoundBooleanSupplier(EnhancedBooleanSupplier{
            clawSubsystem.colorDistSensor.getDistance(DistanceUnit.MM)<35.0
        }).onTrue(Wait(0.1).then(clawSubsystem.closeClaw))


        BoundBooleanSupplier(EnhancedBooleanSupplier{
            abs(Mercurial.gamepad2.leftStickX.state)>0.4})
            .whileTrue(clawSubsystem.rotateClaw)


//        BoundBooleanSupplier(EnhancedBooleanSupplier{
//            abs(Mercurial.gamepad2.rightStickY.state)>0.1})
//            .onTrue(linearSlides.manualControl)
//            .onFalse(linearSlides.holdPose)

//        Mercurial.gamepad2.a.onTrue(
//            linearSlides.closeSlides
//        )

        BoundDoubleSupplier{abs(Mercurial.gamepad1.leftStickX.state)+abs(Mercurial.gamepad1.leftStickY.state)+
        abs(Mercurial.gamepad1.rightStickX.state)
        }.conditionalBindState().greaterThan(0.1).bind().whileTrue(driveSubsystem.driveCommand)

        Mercurial.gamepad2.x.onTrue(
            extendoOpenCommand
        )
            .toggleFalse(extendoCloseCommand)

    }



override fun loop() {
    telemetry.addData("claw servo", clawSubsystem.clawServo.position)
    telemetry.addData("arm", armClawSubsystem.armClawServo.position)
    telemetry.addData("agl", armClawSubsystem.angleClawServo.position)
    telemetry.addData("clawRotationServo", clawSubsystem.clawRotationServo.position)
    telemetry.addData("x", Mercurial.gamepad2.x.state)
    telemetry.addData("lx", Mercurial.gamepad1.leftStickX.state)
    telemetry.addData("ly", Mercurial.gamepad1.leftStickY.state)
    telemetry.addData("rx", Mercurial.gamepad1.rightStickX.state)
//    telemetry.addData("", clawSubsystem.colorDistSensor.getDistance(DistanceUnit.MM))
//    telemetry.addData("", extendoCommand.currentExtendoState)

    telemetry.update()

    }


}
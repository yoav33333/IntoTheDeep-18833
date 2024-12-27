package org.firstinspires.ftc.teamcode.opModes

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.frozenmilk.dairy.core.util.supplier.logical.EnhancedBooleanSupplier
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.bindings.BoundBooleanSupplier
import dev.frozenmilk.mercurial.commands.util.Wait
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.commands.extendoCommand
import org.firstinspires.ftc.teamcode.commands.extendoCommand.extendoCloseCommand
import org.firstinspires.ftc.teamcode.commands.extendoCommand.extendoMacro
import org.firstinspires.ftc.teamcode.subsystems.BulkReads
import org.firstinspires.ftc.teamcode.subsystems.armClawSubsystem
import org.firstinspires.ftc.teamcode.subsystems.clawSubsystem
import org.firstinspires.ftc.teamcode.subsystems.clawSubsystem.rotateClawL
import org.firstinspires.ftc.teamcode.subsystems.clawSubsystem.rotateClawR
import org.firstinspires.ftc.teamcode.subsystems.deposit
import org.firstinspires.ftc.teamcode.subsystems.deposit.depoArmServo
import org.firstinspires.ftc.teamcode.subsystems.deposit.intakeCommand
import org.firstinspires.ftc.teamcode.subsystems.deposit.release
import org.firstinspires.ftc.teamcode.subsystems.deposit.transferCommand
import org.firstinspires.ftc.teamcode.subsystems.driveSubsystem
import org.firstinspires.ftc.teamcode.subsystems.extendoSubsystem
import org.firstinspires.ftc.teamcode.subsystems.linearSlides
import kotlin.math.abs

@BulkReads.Attach
@Mercurial.Attach
@clawSubsystem.Attach
@driveSubsystem.Attach
@extendoCommand.Attach
@linearSlides.Attach
@armClawSubsystem.Attach
@extendoSubsystem.Attach
@deposit.Attach
@TeleOp

class simpleOpMode : OpMode() {

    override fun init() {
        //operator controls
        Mercurial.gamepad2.y.onTrue(clawSubsystem.changeClawPos)
        Mercurial.gamepad2.b.onTrue(release)
        Mercurial.gamepad2.leftStickButton.onTrue(clawSubsystem.resetAngleClaw)
        Mercurial.gamepad2.a.onTrue(intakeCommand)
        Mercurial.gamepad2.dpadDown.whileTrue(extendoSubsystem.moveManualO)
        Mercurial.gamepad2.dpadUp.whileTrue(extendoSubsystem.moveManualC)
        Mercurial.gamepad2.leftBumper.onTrue(rotateClawL)
        Mercurial.gamepad2.rightBumper.onTrue(rotateClawR)
        Mercurial.gamepad2.x.onTrue(extendoMacro)

        BoundBooleanSupplier(EnhancedBooleanSupplier{abs(Mercurial.gamepad2.rightStickY.state)>0.1})
            .whileTrue(linearSlides.manualControl)
        BoundBooleanSupplier(EnhancedBooleanSupplier{clawSubsystem.colorDistSensor.getDistance(DistanceUnit.MM)<35.0 && clawSubsystem.check})
            .onTrue(Wait(0.1).then(clawSubsystem.closeClaw))


//        Mercurial.gamepad2.a.onTrue(linearSlides.closeSlides)
        //drive controls
        Mercurial.gamepad1.a.onTrue(release)
        Mercurial.gamepad1.b.onTrue(driveSubsystem.gears)
        Mercurial.gamepad1.x.onTrue(driveSubsystem.gears)

    }


override fun loop() {
    telemetry.addData("claw servo", clawSubsystem.clawServo.position)
    telemetry.addData("arm", armClawSubsystem.armClawServo.position)
    telemetry.addData("agl", armClawSubsystem.angleClawServo.position)
    telemetry.addData("clawRotationServo", clawSubsystem.clawRotationServo.position)
    telemetry.addData("p", armClawSubsystem.armClawServo.position)
    telemetry.addData("ry", Mercurial.gamepad2.rightStickY.state)
    telemetry.addData("time", runtime)
    telemetry.addData("pos", depoArmServo.position)
    telemetry.addData("dist", clawSubsystem.colorDistSensor.getDistance(DistanceUnit.MM))
    telemetry.addData("v4b", armClawSubsystem.armClawServo.position)
    telemetry.addData("v4b", clawSubsystem.clawRotationServo.position)
    telemetry.addData("dis dep", deposit.colorSensor.getDistance(DistanceUnit.MM))
    telemetry.addData("flip servo", depoArmServo.position)
    telemetry.addData("ex", extendoSubsystem.extendoServoL.position)
    telemetry.addData("trans", Mercurial.isScheduled(transferCommand))
    telemetry.addData("trans", Mercurial.isScheduled(intakeCommand))

    telemetry.update()

    }

}
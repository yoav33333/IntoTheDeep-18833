package org.firstinspires.ftc.teamcode.subsystems.lift

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DigitalChannel
import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.util.OpModeLazyCell
import dev.frozenmilk.dairy.core.util.supplier.logical.EnhancedBooleanSupplier
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.bindings.BoundBooleanSupplier
import dev.frozenmilk.mercurial.commands.Command
import dev.frozenmilk.mercurial.subsystems.SDKSubsystem
import dev.frozenmilk.mercurial.subsystems.Subsystem
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftVariables.basketState
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftVariables.d
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftVariables.g
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftVariables.liftState
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftVariables.p
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftVariables.targetPosition

import org.firstinspires.ftc.teamcode.util.BetterDigitalChannel
import org.firstinspires.ftc.teamcode.util.Encoder
import org.firstinspires.ftc.teamcode.util.Motor
import org.firstinspires.ftc.teamcode.util.SquIDController
import org.firstinspires.ftc.teamcode.util.SuperMotor
import org.firstinspires.ftc.teamcode.util.controller.PDController
import java.lang.annotation.Inherited

object LiftHardware : SDKSubsystem() {
    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and
            SingleAnnotation(Mercurial.Attach::class.java)

    @Target(AnnotationTarget.CLASS)
    @Retention(AnnotationRetention.RUNTIME)
    @MustBeDocumented
    @Inherited
    annotation class Attach

    val leftCenter by OpModeLazyCell{SuperMotor("lift 4")
        .setMode(RunMode.RUN_WITHOUT_ENCODER)
        .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        .get()}

    val leftSide by OpModeLazyCell{SuperMotor("lift 3")
        .setMode(RunMode.RUN_WITHOUT_ENCODER)
        .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        .get()}

    val rightCenter by OpModeLazyCell{SuperMotor("lift 2")
        .setMode(RunMode.RUN_WITHOUT_ENCODER)
        .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        .setDirection(DcMotorSimple.Direction.REVERSE)
        .get()}

    val rightSide by OpModeLazyCell{SuperMotor("lift 1")
        .setMode(RunMode.RUN_WITHOUT_ENCODER)
        .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        .setDirection(DcMotorSimple.Direction.REVERSE)
        .get()}

    val encoder: Encoder by OpModeLazyCell{
        Encoder("lift 1", DcMotorSimple.Direction.REVERSE)
    }

    val magneticLimit: DigitalChannel by OpModeLazyCell {
        val s = BetterDigitalChannel("magneticLimit")
        s.mode = DigitalChannel.Mode.INPUT
        s
    }

    fun setPower(power: Double) {
        leftCenter.setPower(-power)
        leftSide.setPower(-power)
        rightSide.setPower(power)
        rightCenter.setPower(power)
    }

    fun runToPosition(target: Double){
        setPower(PDController(p,d).calculate(
            getPose().toDouble(), target) + g)
    }
    @JvmStatic
    fun getPose(): Int{
        return encoder.getPose()
    }
    @JvmStatic
    fun setPose(pose: Int){
        encoder.setPose(pose)
    }


    override fun postUserInitHook(opMode: Wrapper) {

        BoundBooleanSupplier(EnhancedBooleanSupplier { !magneticLimit.state })
            .whileTrue(LiftCommands.resetHeight)
        setPose(LiftVariables.startingPose)

    }

    override fun postUserStartHook(opMode: Wrapper) {
//        defaultCommand = LiftCommands.PIDCommand
        LiftCommands.PIDCommand.schedule()
    }

    override fun cleanup(opMode: Wrapper) {
        LiftCommands.PIDCommand.cancel()
    }
}
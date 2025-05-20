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

    val leftCenter = SuperMotor("l1")
        .setMode(RunMode.RUN_WITHOUT_ENCODER)
        .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        .get()

    val leftSide = SuperMotor("l2")
        .setMode(RunMode.RUN_WITHOUT_ENCODER)
        .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        .get()

    val rightCenter = SuperMotor("l3")
        .setMode(RunMode.RUN_WITHOUT_ENCODER)
        .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        .setDirection(DcMotorSimple.Direction.REVERSE)
        .get()

    val rightSide = SuperMotor("l4")
        .setMode(RunMode.RUN_WITHOUT_ENCODER)
        .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        .setDirection(DcMotorSimple.Direction.REVERSE)
        .get()

    val encoder: Encoder by OpModeLazyCell{
        Encoder("l4", DcMotorSimple.Direction.REVERSE)
    }

    val magneticLimit: DigitalChannel by OpModeLazyCell {
        val s = BetterDigitalChannel("magneticLimit")
        s.mode = DigitalChannel.Mode.INPUT
        s
    }

    /**
     * Sets the power level for all lift motors, applying the specified value to the left motors and the inverted value to the right motors.
     *
     * @param power The power level to apply to the lift motors, where positive values raise and negative values lower the lift.
     */
    fun setPower(power: Double) {
        leftCenter.setPower(power)
        leftSide.setPower(power)
        rightSide.setPower(-power)
        rightCenter.setPower(-power)
    }

    /**
     * Moves the lift toward the target position using PID control with gravity compensation.
     *
     * Calculates the required motor power based on the current encoder position, the target position, and a gravity compensation term, then applies this power to the lift motors.
     */
    fun runToPosition(){
        setPower(SquIDController(p).calculate(
            getPose().toDouble(), targetPosition.toDouble()) + g)
    }

    /**
     * Returns the current position of the lift as measured by the encoder.
     *
     * @return The encoder's current pose value.
     */
    fun getPose(): Int{
        return encoder.getPose()
    }

    /**
     * Sets the encoder's current pose to the specified value.
     *
     * @param pose The new pose value to assign to the encoder.
     */
    fun setPose(pose: Int){
        encoder.setPose(pose)
    }

    override var defaultCommand: Command? = LiftCommands.PIDCommand

    /**
     * Performs pre-initialization setup for the lift subsystem before user code runs.
     *
     * Binds the lift reset command to the magnetic limit switch, initializes the basket and lift states,
     * and sets the encoder pose to the starting position.
     *
     * @param opMode The current operational mode context.
     */
    override fun preUserInitHook(opMode: Wrapper) {

        BoundBooleanSupplier(EnhancedBooleanSupplier { !magneticLimit.state })
            .whileTrue(LiftCommands.resetHeight)
        basketState = BasketState.HIGH
        liftState = LiftState.AUTO
        setPose(LiftVariables.startingPose)

    }
}
package org.firstinspires.ftc.teamcode.subsystems.robot

import dev.frozenmilk.dairy.core.Feature
import dev.frozenmilk.dairy.core.FeatureRegistrar.activeOpModeWrapper
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.subsystems.Subsystem
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmTarget
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmVariables.armTarget
import org.firstinspires.ftc.teamcode.subsystems.extendo.ExtendoState
import org.firstinspires.ftc.teamcode.subsystems.extendo.ExtendoVariables.extendoState
import org.firstinspires.ftc.teamcode.subsystems.lift.BasketState
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftState
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftVariables.basketState
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftVariables.liftState
import org.firstinspires.ftc.teamcode.subsystems.robot.RobotCommands.macro
import org.firstinspires.ftc.teamcode.subsystems.robot.RobotVariables.gameElement
import org.firstinspires.ftc.teamcode.subsystems.robot.RobotVariables.transferState

import java.lang.annotation.Inherited

object Robot: Feature{
    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and
            SingleAnnotation(Mercurial.Attach::class.java)

    @Target(AnnotationTarget.CLASS)
    @Retention(AnnotationRetention.RUNTIME)
    @MustBeDocumented
    @Inherited
    annotation class Attach

    fun initRobotState(){
        liftState = LiftState.AUTO
        basketState = BasketState.HIGH
        gameElement = GameElement.SAMPLE
        transferState = TransferState.TRANSFER
        extendoState = ExtendoState.FULL_OPEN
        armTarget = ArmTarget.LOW
    }

    fun initRobotCommands(){
        if (activeOpModeWrapper.opModeType == OpModeMeta.Flavor.TELEOP) {
            macro.reset()
            macro.schedule()
        }
        else{

        }
    }

    override fun preUserInitHook(opMode: Wrapper) {
        initRobotState()
        initRobotCommands()
    }
}
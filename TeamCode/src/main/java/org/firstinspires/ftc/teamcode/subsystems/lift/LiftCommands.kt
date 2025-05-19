package org.firstinspires.ftc.teamcode.subsystems.lift

import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.commands.groups.Sequential
import org.firstinspires.ftc.teamcode.commands.util.InstantCommand
import org.firstinspires.ftc.teamcode.commands.util.WaitUntil
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmCommands.avoidBasket
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmCommands.smartDeposit
import org.firstinspires.ftc.teamcode.subsystems.depositClaw.DepositClawCommands.quickRC
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftHardware.getPose
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftHardware.magneticLimit
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftHardware.setPower
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftVariables.basketState
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftVariables.highBasketPose
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftVariables.highChamberPoseDown
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftVariables.highChamberPoseUp
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftVariables.liftState
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftVariables.lowBasketPose
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftVariables.targetPosition

import org.firstinspires.ftc.teamcode.subsystems.robot.GameElement
import org.firstinspires.ftc.teamcode.subsystems.robot.RobotVariables
import kotlin.math.abs

object LiftCommands {
    val PIDCommand = Lambda("LiftPIDCommand")
        .setRunStates(Wrapper.OpModeState.ACTIVE, Wrapper.OpModeState.INIT)
        .setInit{
            LiftVariables.liftState = LiftState.AUTO
        }
        .setExecute{
            if (liftState == LiftState.AUTO) {
                LiftHardware.runToPosition()
            }
        }
        .setFinish{false}
        .addRequirements(LiftHardware)
        .setInterruptible(true)

    val manualControl = Lambda("manualLiftControl")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setExecute{
            setPower(Mercurial.gamepad2.rightStickY.state)
        }
        .setFinish{
            Mercurial.gamepad2.rightStickY.state<0.2
        }
        .addRequirements(LiftHardware)

    val resetHeight = Lambda("resetHeight")
        .setInit { LiftHardware.setPose(getPose()) }

    /**
         * Creates a command to set the lift's target position to the specified goal.
         *
         * @param goal The desired lift position to move to.
         * @return A Lambda command that sets the lift's target position when initialized.
         */
        fun goToPreset(goal: Int) = Lambda("goToPreset")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit { targetPosition = goal }

    val switchBasket = Lambda("SWB")
        .setInit{
            if (LiftVariables.basketState == BasketState.HIGH) {
                LiftVariables.basketState = BasketState.LOW
            } else {
                LiftVariables.basketState = BasketState.HIGH
            }
            avoidBasket.schedule()
            goToBasket.schedule()
            Sequential(
                WaitUntil{(abs(Mercurial.gamepad2.rightStickY.state) >0.2 ||(targetPosition >500 && targetPosition - 1000 <getPose())
                        || (RobotVariables.gameElement == GameElement.SPECIMEN && getPose()>1000) )},
                smartDeposit
            ).schedule()
        }

    val goToBasket = Lambda("GTB")
        .setInit{
            if (basketState == BasketState.HIGH) {
                goToHighBasket.schedule()
            }
            else{
                goToLowBasket.schedule()
            }
        }

    @JvmStatic
    val closeSlides = Lambda("close slides")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit{
            targetPosition = 0
            liftState = LiftState.DISABLED
            setPower(-1.0)
        }
        .setFinish{!magneticLimit.state}
        .addRequirements(LiftHardware)

    @JvmStatic val enablePID = Lambda("enablePID")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit{
            liftState = LiftState.AUTO
        }

    @JvmStatic
    val goToHighBasket = goToPreset(highBasketPose)
        .addInit{RobotVariables.gameElement = GameElement.SAMPLE}

    @JvmStatic
    val goToLowBasket = goToPreset(lowBasketPose)
        .addInit{RobotVariables.gameElement = GameElement.SAMPLE}
    val up = Lambda("up")
        .setInit{
            targetPosition += 19000
        }
    @JvmStatic
    val goToHighChamberUp = goToPreset(highChamberPoseUp)
        .addInit {
            RobotVariables.gameElement = GameElement.SPECIMEN
            quickRC { true }.schedule()
        }

    @JvmStatic
    val goToHighChamberDown = goToPreset(highChamberPoseDown)
        .addInit {
            RobotVariables.gameElement = GameElement.SPECIMEN
            quickRC { true }.schedule()

        }


}
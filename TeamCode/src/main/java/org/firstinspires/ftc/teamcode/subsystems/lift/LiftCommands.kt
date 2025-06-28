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
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftVariables.closedPose
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftVariables.highBasketPose
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftVariables.highChamberPose
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftVariables.highChamberPoseDown
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftVariables.highChamberPoseUp
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftVariables.liftState
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftVariables.lockedTargetPosition
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftVariables.lowBasketPose
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftVariables.targetPosition
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftVariables.wallPose

import org.firstinspires.ftc.teamcode.subsystems.robot.GameElement
import org.firstinspires.ftc.teamcode.subsystems.robot.RobotVariables
import java.util.function.IntSupplier
import kotlin.math.abs

object LiftCommands {
    val PIDCommand = Lambda("LiftPIDCommand")
        .setRunStates(Wrapper.OpModeState.ACTIVE, Wrapper.OpModeState.INIT)
        .setInit{
            liftState = LiftState.AUTO
        }
        .setExecute{
            when (liftState) {
                LiftState.AUTO -> LiftHardware.runToPosition(targetPosition.toDouble())
                LiftState.LOCKED -> LiftHardware.runToPosition(lockedTargetPosition.toDouble())
                LiftState.MANUAL -> setPower(Mercurial.gamepad2.rightStickY.state)
            }
        }
        .setFinish{false}


    val manualControl = Lambda("manualLiftControl")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit{ liftState = LiftState.MANUAL}
        .setFinish{
            abs(Mercurial.gamepad2.rightStickY.state) <0.2
        }
        .setEnd{
            setPower(0.0)
            liftState = LiftState.AUTO
        targetPosition = getPose()
        }



    val resetHeight = Lambda("resetHeight")
        .setInit { LiftHardware.setPose(0) }

    fun goToPreset(goal: IntSupplier) = Lambda("goToPreset")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit { targetPosition = goal.asInt }

    val switchBasket = Lambda("SWB")
        .setInit{
            basketState =
                if (basketState == BasketState.HIGH) {
                    BasketState.LOW
                } else {
                    BasketState.HIGH
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
            when (basketState){
                BasketState.HIGH -> goToHighBasket.schedule()
                BasketState.LOW -> goToLowBasket.schedule()
            }
//            if (basketState == BasketState.HIGH) {
//                goToHighBasket.schedule()
//            }
//            else{
//                goToLowBasket.schedule()
//            }
        }

    @JvmStatic
    val closeSlides = goToPreset { closedPose }
        .addInit{
            liftState = LiftState.LOCKED
            lockedTargetPosition = closedPose
            targetPosition = closedPose
        }

    @JvmStatic
    val goToLowBasket = goToPreset { closedPose }

//        .addInit{RobotVariables.gameElement = GameElement.SAMPLE}
//        .setInit{
//            liftState = LiftState.DISABLED
//            targetPosition = 0
//            liftState = LiftState.DISABLED
////            if (!magneticLimit.state) {
//                setPower(-1.0)
////            }
//        }
//        .setFinish{!magneticLimit.state}
//        .setEnd{ liftState = LiftState.AUTO
//            setPower(0.0)
//        }

    @JvmStatic val enablePID = Lambda("enablePID")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit{
            liftState = LiftState.AUTO
        }

    @JvmStatic
    val goToHighBasket = goToPreset { highBasketPose }
        .addInit{RobotVariables.gameElement = GameElement.SAMPLE}

//    @JvmStatic
//    val goToLowBasket = goToPreset { lowBasketPose }
//        .addInit{RobotVariables.gameElement = GameElement.SAMPLE}

    @JvmStatic
    val goToHighChamberUp = goToPreset { highChamberPoseUp }
        .addInit {
            RobotVariables.gameElement = GameElement.SPECIMEN
            quickRC { true }.schedule()
        }
    @JvmStatic
    val goToHighChamber = goToPreset { highChamberPose }
        .addInit {
            RobotVariables.gameElement = GameElement.SPECIMEN
            quickRC { true }.schedule()
        }
    @JvmStatic
    val goToWall = goToPreset { wallPose }

    @JvmStatic
    val goToHighChamberDown = goToPreset { highChamberPoseDown }
        .addInit {
            RobotVariables.gameElement = GameElement.SPECIMEN
            quickRC { true }.schedule()

        }

//    val down = Lambda("down")
//        .setInit{ targetPosition-=10000}


}
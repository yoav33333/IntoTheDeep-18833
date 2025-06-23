package org.firstinspires.ftc.teamcode.subsystems.lift

import com.acmerobotics.dashboard.config.Config

@Config
object LiftVariables {
    /*PID*/
    @JvmField var p = 0.000055
    @JvmField var i = 0
    @JvmField var d = 0.9
    /*gravity*/
    @JvmField var g = 0.12
    /*lift control*/
    @JvmField var liftState = LiftState.AUTO
    @JvmField var basketState = BasketState.HIGH
    @JvmField var startingPose = 0
    @JvmField var offset = 0
    @JvmField var targetPosition = 0
    @JvmField var lockedTargetPosition = 0
    /*presets*/
    @JvmField var lowBasketPose = 40000
    @JvmField var closedPose = 26000
    @JvmField var lowChamberPose = 0
    @JvmField var highBasketPose = 53000
    @JvmField var highChamberPoseUp = 31500
    @JvmField var highChamberPoseDown = 45000
    @JvmField var highChamberPose = 38000
    @JvmField var wallPose = 0

}
enum class LiftState{
    AUTO,
    LOCKED,
    MANUAL,
}

enum class BasketState{
    LOW,
    HIGH
}

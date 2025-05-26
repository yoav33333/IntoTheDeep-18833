package org.firstinspires.ftc.teamcode.subsystems.lift

import com.acmerobotics.dashboard.config.Config

@Config
object LiftVariables {
    /*PID*/
    @JvmField var p = 0.000095
    @JvmField var i = 0
    @JvmField var d = 0.9
    /**gravity**/
    @JvmField var g = 0.14
    /*lift control*/
    @JvmField var liftState = LiftState.AUTO
    @JvmField var basketState = BasketState.HIGH
    @JvmField var startingPose = 0
    @JvmField var offset = 0
    @JvmField var targetPosition = 0
    /*presets*/
    @JvmField var lowBasketPose = 40000
    @JvmField var lowChamberPose = 0
    @JvmField var highBasketPose = 80000
    @JvmField var highChamberPoseUp = 31500
    @JvmField var highChamberPoseDown = 45000

}
enum class LiftState{
    AUTO,
    DISABLED,
    MANUAL,
}

enum class BasketState{
    LOW,
    HIGH
}

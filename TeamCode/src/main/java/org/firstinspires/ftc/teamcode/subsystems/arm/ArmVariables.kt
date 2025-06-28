package org.firstinspires.ftc.teamcode.subsystems.arm

import com.acmerobotics.dashboard.config.Config

@Config
object ArmVariables {
    /*extension*/
    @JvmField var closeExtensionPosition = 0.0
    @JvmField var openExtensionPosition = 0.75
    @JvmField var transferExtensionPosition = 0.75
    @JvmField var depositExtensionPosition = 0.75
    @JvmField var depositHighExtensionPosition = 0.75
    @JvmField var avoidBasketExtensionPosition = 0.75
    @JvmField var wallExtensionPosition = 0.75
    @JvmField var chamberExtensionPosition = 0.75
    /*arm*/
    @JvmField var transferArmPosition = 0.931
    @JvmField var depositArmPosition = 0.29
    @JvmField var depositHighArmPosition = 0.29
    @JvmField var slamArmPosition = 0.08
    @JvmField var avoidBasketPosition = 0.39
    @JvmField var armLowBasket = 0.19
    @JvmField var wallArmPosition = 0.74
    @JvmField var chamberArmPosition = 0.08
    @JvmField var armTarget = ArmTarget.LOW
}
enum class ArmTarget{
    HIGH,
    LOW
}
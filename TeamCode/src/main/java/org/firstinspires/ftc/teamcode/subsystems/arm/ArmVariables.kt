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
    @JvmField var transferArmPosition = 0.83
    @JvmField var depositArmPosition = 0.16
    @JvmField var depositHighArmPosition = 0.2
    @JvmField var slamArmPosition = 0.0
    @JvmField var avoidBasketPosition = 0.4
    @JvmField var wallArmPosition = 0.62
    @JvmField var chamberArmPosition = 0.01
    @JvmField var armTarget = ArmTarget.LOW
}
enum class ArmTarget{
    HIGH,
    LOW
}
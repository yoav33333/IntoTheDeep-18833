package org.firstinspires.ftc.teamcode.subsystems.arm

import com.acmerobotics.dashboard.config.Config

@Config
object ArmVariables {
    /*extension*/
//    @JvmField var transferExtensionPosition = 0.0
//    @JvmField var frontDepositExtensionPosition = 0.0
//    @JvmField var backDepositExtensionPosition = 0.0
//    @JvmField var basketDepositExtensionPosition = 0.0
//    @JvmField var wallIntakeExtensionPosition = 0.0
    /*arm*/
    @JvmField var transferArmPosition = 0.01
    @JvmField var DepositArmPosition = 0.71
    @JvmField var DepositHighArmPosition = 0.1
    @JvmField var SlamArmPosition = 0.9
    @JvmField var avoidBasketPosition = 0.43
    @JvmField var armTarget = ArmTarget.LOW
//    @JvmField var backDepositArmPosition = 0.0
//    @JvmField var basketDepositArmPosition = 0.0
//    @JvmField var wallIntakeArmPosition = 0.0
}
enum class ArmTarget{
    HIGH,
    LOW
}
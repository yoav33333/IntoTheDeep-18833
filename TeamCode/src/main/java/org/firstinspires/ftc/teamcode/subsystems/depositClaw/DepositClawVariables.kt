package org.firstinspires.ftc.teamcode.subsystems.depositClaw

import com.acmerobotics.dashboard.config.Config

@Config
object DepositClawVariables {
    /*claw*/
    @JvmField var closedClawPosition = 0.0
    @JvmField var openedClawPosition = 1.0
    @JvmField var semiClosedClawPosition = 0.4
    /*color sensor*/
    @JvmField var minDistance = 35
}
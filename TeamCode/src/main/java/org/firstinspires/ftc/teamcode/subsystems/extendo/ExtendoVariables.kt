package org.firstinspires.ftc.teamcode.subsystems.extendo

import com.acmerobotics.dashboard.config.Config

@Config
object ExtendoVariables {
    @JvmField var openPosition = 0.38
    @JvmField var partialOpeningPosition = 0.75
    @JvmField var closePosition = 0.99
    @JvmField var extendoSpeed = 0.04
    @JvmField var extendoState = ExtendoState.FULL_OPEN
}

enum class ExtendoState{
    FULL_OPEN,
    PARTIAL_OPEN,
}
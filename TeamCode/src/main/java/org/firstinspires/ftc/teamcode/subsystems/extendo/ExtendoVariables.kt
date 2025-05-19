package org.firstinspires.ftc.teamcode.subsystems.extendo

import com.acmerobotics.dashboard.config.Config

@Config
object ExtendoVariables {
    @JvmField var openPosition = 0.33
    @JvmField var partialOpeningPosition = 0.75
    @JvmField var closePosition = 0.99
    @JvmField var extendoState = ExtendoState.FULL_OPEN
}

enum class ExtendoState{
    FULL_OPEN,
    PARTIAL_OPEN,
}
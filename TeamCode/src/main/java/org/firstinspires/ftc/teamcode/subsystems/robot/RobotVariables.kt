package org.firstinspires.ftc.teamcode.subsystems.robot

import com.acmerobotics.dashboard.config.Config

@Config
object RobotVariables {
    @JvmField var gameElement = GameElement.SAMPLE
    @JvmField var transferState = TransferState.TRANSFER
    @JvmField var deltaToReopenAfterSwitch = 16000
}

enum class GameElement{
    SAMPLE,
    SPECIMEN
}

enum class TransferState{
    DO_NOTHING,
    TRANSFER,
}

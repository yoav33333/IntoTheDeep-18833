package org.firstinspires.ftc.teamcode.subsystems.antony

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.hardware.rev.RevBlinkinLedDriver

@Config
object AntonyVariables {
    @JvmField var default = RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_FOREST_PALETTE
    @JvmField var endGame = RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_RAINBOW_PALETTE
    @JvmField var lowBattery = RevBlinkinLedDriver.BlinkinPattern.RED

    @JvmField var minVoltThreshold = 9.0
}
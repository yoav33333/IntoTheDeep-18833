package org.firstinspires.ftc.teamcode.subsystems.v4b

import com.acmerobotics.dashboard.config.Config

@Config
object V4bVariables {
    /*arm*/
    @JvmField var armOutPosition = 0.0
    @JvmField var armOutPositionAuto = 0.19
    @JvmField var armOutPositionTele = 0.19
    @JvmField var armInPosition = 0.45
    @JvmField var armPushPosition = 0.0
    @JvmField var armUpPosition = 0.15
    /*pitch*/
    @JvmField var transferPosition = 0.38
    @JvmField var wallPosition = 0.0
    @JvmField var postTransferPosition = 0.5
    @JvmField var intakePosition = 1.0
    @JvmField var pitchPushPosition = 0.8
}
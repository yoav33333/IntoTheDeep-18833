package org.firstinspires.ftc.teamcode.opModes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import dev.frozenmilk.dairy.core.FeatureRegistrar

open abstract class linearOpMode_dairy : LinearOpMode() {
    override fun runOpMode() {
        /* start */
        FeatureRegistrar.opModePreStart(FeatureRegistrar.activeOpModeWrapper)
        myStart()
        FeatureRegistrar.opModePostStart(FeatureRegistrar.activeOpModeWrapper)

        /* init */
        FeatureRegistrar.opModePreInit(FeatureRegistrar.activeOpModeWrapper)
        myInit()
        while (opModeInInit()) {
            FeatureRegistrar.opModePreInitLoop(FeatureRegistrar.activeOpModeWrapper)
            myInitLoop()
            FeatureRegistrar.opModePostInitLoop(FeatureRegistrar.activeOpModeWrapper)
        }
        FeatureRegistrar.opModePostInit(FeatureRegistrar.activeOpModeWrapper)

        /* loop */
        FeatureRegistrar.opModePreLoop(FeatureRegistrar.activeOpModeWrapper)
        while (opModeIsActive()) {
            FeatureRegistrar.opModePreLoop(FeatureRegistrar.activeOpModeWrapper)
            myLoop()
            FeatureRegistrar.opModePostLoop(FeatureRegistrar.activeOpModeWrapper)
        }

        /* stop */
        FeatureRegistrar.opModePreStop(FeatureRegistrar.activeOpModeWrapper)
        myStop()
        FeatureRegistrar.opModePostStop(FeatureRegistrar.activeOpModeWrapper)

    }
    open fun myStart(){}
    open fun myInit(){}
    open fun myInitLoop(){}
    open fun myLoop(){}
    open fun myStop(){}


}
package org.firstinspires.ftc.teamcode.opModes

import com.qualcomm.hardware.lynx.commands.core.LynxI2cConfigureChannelCommand
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import dev.frozenmilk.dairy.core.Feature
import dev.frozenmilk.dairy.core.FeatureRegistrar

open class CommandOpMode(vararg var features: Feature) : LinearOpMode() {
    override fun runOpMode() {

        FeatureRegistrar.checkFeatures(*features)
        /*init*/
        FeatureRegistrar.opModePreInit(FeatureRegistrar.activeOpModeWrapper)
        preInit()
        myInit()
        FeatureRegistrar.opModePostInit(FeatureRegistrar.activeOpModeWrapper)
        /*init_loop*/
        while (opModeInInit()) {
            FeatureRegistrar.opModePreInitLoop(FeatureRegistrar.activeOpModeWrapper)
            backgroundLoop()
            myInitLoop()
            FeatureRegistrar.opModePostInitLoop(FeatureRegistrar.activeOpModeWrapper)
        }
        waitForStart()
        /*start*/
        FeatureRegistrar.opModePreStart(FeatureRegistrar.activeOpModeWrapper)
        myStart()
        FeatureRegistrar.opModePostStart(FeatureRegistrar.activeOpModeWrapper)
        /*loop*/
        while (opModeIsActive()) {
            FeatureRegistrar.opModePreLoop(FeatureRegistrar.activeOpModeWrapper)
            myLoop()
            FeatureRegistrar.opModePostLoop(FeatureRegistrar.activeOpModeWrapper)
        }
        /*stop*/
        FeatureRegistrar.opModePreStop(FeatureRegistrar.activeOpModeWrapper)
        myStop()
        FeatureRegistrar.opModePostStop(FeatureRegistrar.activeOpModeWrapper)
        myFullStop()

    }
    /**don't use!!!**/
    open fun preInit() {}
    open fun backgroundLoop(){}
    open fun myInit() {}
    open fun myStart() {}
    open fun myInitLoop() {}
    open fun myLoop() {}
    open fun myStop() {}
    open fun myFullStop() {}


}
package org.firstinspires.ftc.teamcode.opModes

import com.qualcomm.hardware.lynx.commands.core.LynxI2cConfigureChannelCommand
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import dev.frozenmilk.dairy.core.Feature
import dev.frozenmilk.dairy.core.FeatureRegistrar

open class CommandOpMode(vararg var features: Feature) : LinearOpMode() {
    /**
     * Manages the complete lifecycle of the operation mode, invoking feature hooks and user-overridable methods at each stage.
     *
     * This method coordinates initialization, pre-start, main loop, and shutdown phases, ensuring that registered features and user-defined logic are executed in the correct order.
     */
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
    /**
 * Called repeatedly during the initialization loop phase before the operation mode starts.
 *
 * Override this method to implement custom behavior that should run while the op mode is in the initialization loop.
 */
open fun myInitLoop() {}
    /**
 * Called repeatedly during the main operation mode loop after the start signal.
 *
 * Override this method to define custom robot behavior that should execute continuously while the operation mode is active.
 */
open fun myLoop() {}
    /**
 * Called during the stop phase of the operation mode lifecycle.
 *
 * Override this method to implement custom shutdown or cleanup logic when the operation mode ends.
 */
open fun myStop() {}
    /**
 * Called after the operation mode has fully stopped to perform any final cleanup.
 *
 * Override this method to implement actions that should occur once all other stop procedures are complete.
 */
open fun myFullStop() {}


}
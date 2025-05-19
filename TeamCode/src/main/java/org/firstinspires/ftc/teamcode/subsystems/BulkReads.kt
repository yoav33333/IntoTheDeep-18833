package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.lynx.commands.core.LynxI2cConfigureChannelCommand
import com.qualcomm.robotcore.hardware.configuration.LynxConstants
import dev.frozenmilk.dairy.core.Feature
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.wrapper.Wrapper

object BulkReads : Feature {
    // first, we need to set up the dependency
    // this makes a rule that says:
    // "for this feature to receive updates about an OpMode, it must have @BulkReads.Attach"
    override var dependency: Dependency<*> = SingleAnnotation(Attach::class.java)

    lateinit var modules: List<LynxModule>

    /**
     * Initializes the list of Lynx hardware modules and sets their bulk caching mode to manual before user initialization.
     *
     * @param opMode The current OpMode wrapper providing access to the hardware map.
     */
    override fun preUserInitHook(opMode: Wrapper) {

        // collect and store the modules
        modules = opMode.opMode.hardwareMap.getAll(LynxModule::class.java)
        // set them to manual

        modules.forEach { it.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL }
    }

    override fun preUserInitLoopHook(opMode: Wrapper) {
        modules.forEach { it.clearBulkCache() }
    }

    override fun preUserStartHook(opMode: Wrapper) {
        modules.forEach { it.clearBulkCache() }
    }

    override fun preUserLoopHook(opMode: Wrapper) {
        modules.forEach { it.clearBulkCache() }
    }

    override fun cleanup(opMode: Wrapper) {
        modules = emptyList()
    }

    // the @BulkReads.Attach annotation
    @Target(AnnotationTarget.CLASS)
    @Retention(AnnotationRetention.RUNTIME)
    annotation class Attach
}

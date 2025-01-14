package org.firstinspires.ftc.teamcode.subsystems


import android.content.Context
import android.view.Menu
import com.qualcomm.ftccommon.FtcEventLoop
import com.qualcomm.robotcore.eventloop.opmode.AnnotatedOpModeManager
import com.qualcomm.robotcore.util.WebHandlerManager
import dev.frozenmilk.sinister.apphooks.OnCreate
import dev.frozenmilk.sinister.apphooks.OnCreateEventLoop
import dev.frozenmilk.sinister.apphooks.OnCreateMenu
import dev.frozenmilk.sinister.apphooks.OnDestroy
import dev.frozenmilk.sinister.apphooks.OpModeRegistrar
import dev.frozenmilk.sinister.apphooks.WebHandlerRegistrar

@Suppress("unused")
class tonsOfShit private constructor() {
    // we could do whatever we want here!
    /**
     * this is a private static inner class, to prevent AppHook code from leaking to the public api
     *
     * Sinister's scanning looks for instances to work with. In kotlin, this is an object class, in Java we need to re-create that by hand.
     */
    private object AppHook : OnCreate, OnCreateEventLoop, OnCreateMenu, OnDestroy, OpModeRegistrar,
        WebHandlerRegistrar {
        override fun onCreate(context: Context) {

        }

        override fun onCreateEventLoop(
            context: Context,
            ftcEventLoop: FtcEventLoop
        ) {

        }

        override fun onCreateMenu(context: Context, menu: Menu) {

        }

        override fun onDestroy(context: Context) {

        }

        override fun registerOpModes(opModeManager: AnnotatedOpModeManager) {

        }

        override fun webHandlerRegistrar(
            context: Context,
            webHandlerManager: WebHandlerManager
        ) {

        }


    }
}

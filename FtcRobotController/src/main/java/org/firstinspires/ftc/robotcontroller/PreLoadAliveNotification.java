package org.firstinspires.ftc.robotcontroller;

import org.firstinspires.ftc.robotcore.internal.system.AppAliveNotifier;

import dev.frozenmilk.sinister.loading.Preload;

@Preload
public class PreLoadAliveNotification {
    /** @noinspection InstantiationOfUtilityClass*/
    public static PreLoadAliveNotification INSTANCE = new PreLoadAliveNotification();
    private PreLoadAliveNotification() {
        AppAliveNotifier.getInstance().notifyAppAlive();
    }
}
﻿using Android.App;
using Android.Content.PM;
using Android.OS;

namespace WarehouseApp
{
    [Activity(
    //Theme = "@style/Maui.SplashTheme",
    Theme = "@style/SplashTheme",
    MainLauncher = true, ConfigurationChanges = ConfigChanges.ScreenSize | ConfigChanges.Orientation | ConfigChanges.UiMode | ConfigChanges.ScreenLayout | ConfigChanges.SmallestScreenSize)]
    public class MainActivity : MauiAppCompatActivity
    {
    }
}

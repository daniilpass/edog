plugins {
    id("com.android.application")
}

android {
    namespace = "com.edog.video"
    compileSdk = 35

    defaultConfig {
        applicationId = "com.edog.video"
        minSdk = 26
        targetSdk = 35
        versionCode = 1
        versionName = "1.0"
    }

    buildTypes {
        release {
            isMinifyEnabled = false
            signingConfig = signingConfigs.getByName("debug")
        }
    }
}

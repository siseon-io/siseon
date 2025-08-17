package com.hmh.siseon.siseon2  // ← 네 애플리케이션 패키지명 그대로

import android.bluetooth.BluetoothGatt
import io.flutter.embedding.android.FlutterActivity
import io.flutter.embedding.engine.FlutterEngine
import io.flutter.plugin.common.MethodChannel

class MainActivity : FlutterActivity() {
    // Flutter ↔ Android 통신 채널 이름
    private val CHANNEL = "siseon2/bluetooth"

    override fun configureFlutterEngine(flutterEngine: FlutterEngine) {
        super.configureFlutterEngine(flutterEngine)

        MethodChannel(
            flutterEngine.dartExecutor.binaryMessenger,
            CHANNEL
        ).setMethodCallHandler { call, result ->
            when (call.method) {
                "refreshGatt" -> {
                    // Flutter에서 전달한 GATT 객체(프록시)의 네이티브 GATT 인스턴스를 꺼내야 하지만,
                    // 여기서는 예시로 파라미터 없이 호출해서 그냥 캐시 리프레시 메서드만 수행합니다.
                    val success = refreshDeviceCache()
                    result.success(success)
                }
                else -> result.notImplemented()
            }
        }
    }

    /**
     * 실제로는 BluetoothGatt 인스턴스를 받아야 하지만,
     * flutter_blue_plus 플러그인을 포크/확장하지 않았다면
     * 리플렉션만 호출해도 별 효과가 없을 수 있습니다.
     * 여기서는 예시로 클래스 메서드만 리플렉션으로 호출하는 형태로 보여줍니다.
     */
    private fun refreshDeviceCache(): Boolean {
        return try {
            // BluetoothGatt.refresh() 메서드를 호출
            val refreshMethod = BluetoothGatt::class.java.getMethod("refresh")
            refreshMethod.isAccessible = true
            // 실제로는 gatt 인스턴스를 invoke해야 하지만, 예시용으로 null로 호출
            refreshMethod.invoke(null) as Boolean
        } catch (e: Exception) {
            e.printStackTrace()
            false
        }
    }
}

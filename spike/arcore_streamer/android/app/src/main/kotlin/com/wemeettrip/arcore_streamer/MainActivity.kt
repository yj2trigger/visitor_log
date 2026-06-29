package com.wemeettrip.arcore_streamer

import android.Manifest
import android.content.pm.PackageManager
import android.opengl.GLES20
import android.opengl.GLSurfaceView
import android.os.Bundle
import android.util.Log
import android.view.ViewGroup
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat
import com.google.ar.core.ArCoreApk
import com.google.ar.core.Config
import com.google.ar.core.Session
import com.google.ar.core.TrackingState
import io.flutter.embedding.android.FlutterActivity
import io.flutter.plugin.common.EventChannel
import java.nio.ByteBuffer
import java.nio.ByteOrder
import javax.microedition.khronos.egl.EGLConfig
import javax.microedition.khronos.opengles.GL10

class MainActivity : FlutterActivity() {

    companion object {
        private const val TAG = "ArcoreStreamer"
        private const val CHANNEL = "arcore_depth_stream"
        private const val CAMERA_PERMISSION = Manifest.permission.CAMERA
        private const val CAMERA_PERMISSION_CODE = 1001
    }

    private var session: Session? = null
    private var eventSink: EventChannel.EventSink? = null
    private lateinit var glView: GLSurfaceView
    private var cameraTextureId = -1
    @Volatile private var isProcessing = false

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        EventChannel(flutterEngine!!.dartExecutor.binaryMessenger, CHANNEL)
            .setStreamHandler(object : EventChannel.StreamHandler {
                override fun onListen(args: Any?, sink: EventChannel.EventSink?) {
                    eventSink = sink
                    Log.d(TAG, "Dart 구독 시작")
                }
                override fun onCancel(args: Any?) { eventSink = null }
            })

        glView = GLSurfaceView(this).apply {
            setEGLContextClientVersion(2)
            setRenderer(object : GLSurfaceView.Renderer {
                override fun onSurfaceCreated(gl: GL10?, config: EGLConfig) {
                    val textures = IntArray(1)
                    GLES20.glGenTextures(1, textures, 0)
                    cameraTextureId = textures[0]
                    // requestInstall은 UI 스레드에서 호출
                    runOnUiThread { setupSession() }
                }
                override fun onSurfaceChanged(gl: GL10?, w: Int, h: Int) {}
                override fun onDrawFrame(gl: GL10?) {
                    processFrame()
                    // 30fps 제한으로 과열 방지
                    Thread.sleep(33)
                }
            })
            renderMode = GLSurfaceView.RENDERMODE_CONTINUOUSLY
        }
        addContentView(glView, ViewGroup.LayoutParams(1, 1))

        if (ContextCompat.checkSelfPermission(this, CAMERA_PERMISSION)
            != PackageManager.PERMISSION_GRANTED) {
            ActivityCompat.requestPermissions(this, arrayOf(CAMERA_PERMISSION),
                CAMERA_PERMISSION_CODE)
        }
    }

    private fun setupSession() {
        if (session != null) return
        try {
            if (ArCoreApk.getInstance().requestInstall(this, true) !=
                ArCoreApk.InstallStatus.INSTALLED) return

            session = Session(this).also { s ->
                s.configure(Config(s).apply {
                    depthMode = Config.DepthMode.AUTOMATIC
                    updateMode = Config.UpdateMode.LATEST_CAMERA_IMAGE
                })
                s.setCameraTextureName(cameraTextureId)
                s.resume()
            }
            Log.d(TAG, "ARCore 세션 시작")
        } catch (e: Exception) {
            Log.e(TAG, "ARCore 세션 초기화 실패: $e")
        }
    }

    private fun processFrame() {
        val s = session ?: return
        val sink = eventSink ?: return
        if (isProcessing) return
        isProcessing = true

        try {
            val frame = s.update()
            val camera = frame.camera
            if (camera.trackingState != TrackingState.TRACKING) return

            val depthImg = frame.acquireDepthImage16Bits()
            try {
                val camImg = frame.acquireCameraImage()
                try {
                    val dw = depthImg.width;  val dh = depthImg.height
                    val cw = camImg.width;    val ch = camImg.height

                    val intr = camera.imageIntrinsics
                    val fxCam = intr.focalLength[0];  val fyCam = intr.focalLength[1]
                    val cxCam = intr.principalPoint[0]; val cyCam = intr.principalPoint[1]
                    // depth(160×90) ← camera(cw×ch) 비율로 intrinsics 스케일
                    val scaleX = dw.toFloat() / cw.toFloat()
                    val scaleY = dh.toFloat() / ch.toFloat()
                    val fx = fxCam * scaleX; val fy = fyCam * scaleY
                    val cx = cxCam * scaleX; val cy = cyCam * scaleY
                    val poseMatrix = FloatArray(16).also { camera.pose.toMatrix(it, 0) }

                    Log.d(TAG, "depth: ${dw}x${dh}")

                    // depth: uint16 mm → float32 미터
                    val dBuf     = depthImg.planes[0].buffer.asShortBuffer()
                    val depthOut = ByteBuffer.allocate(dw * dh * 4).order(ByteOrder.LITTLE_ENDIAN)
                    for (i in 0 until dw * dh) {
                        // bit 0~12: 깊이(mm), bit 13~15: 신뢰도 → 신뢰도 비트 마스킹
                        val mm = dBuf.get(i).toInt() and 0x1FFF
                        depthOut.putFloat(mm / 1000.0f)
                    }

                    // rgb: Y plane → grayscale, depth 크기로 리사이즈
                    val yp       = camImg.planes[0]
                    val yBuf     = yp.buffer
                    val yRow     = yp.rowStride; val yPix = yp.pixelStride
                    val rgbBytes = ByteArray(dw * dh * 3)
                    for (dy in 0 until dh) for (dx in 0 until dw) {
                        val lum = yBuf.get((dy * ch / dh) * yRow + (dx * cw / dw) * yPix)
                        val i = (dy * dw + dx) * 3
                        rgbBytes[i] = lum; rgbBytes[i+1] = lum; rgbBytes[i+2] = lum
                    }

                    // header: 4×int32 + 4×float32 + 16×float32 = 16+16+64 = 96 bytes
                    val header = ByteBuffer.allocate(96).order(ByteOrder.LITTLE_ENDIAN).apply {
                        putInt(dw); putInt(dh); putInt(dw); putInt(dh)
                        putFloat(fx); putFloat(fy); putFloat(cx); putFloat(cy)
                        poseMatrix.forEach { putFloat(it) }
                    }.array()

                    val bundle = header + depthOut.array() + rgbBytes
                    runOnUiThread { sink.success(bundle) }
                    Log.d(TAG, "전송: ${dw}x${dh}")

                } finally {
                    camImg.close()
                }
            } finally {
                depthImg.close()
            }
        } catch (e: Exception) {
            Log.e(TAG, "processFrame 오류", e)  // 스택 트레이스 포함
        } finally {
            isProcessing = false
        }
    }

    override fun onRequestPermissionsResult(
        requestCode: Int, permissions: Array<String>, grantResults: IntArray
    ) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults)
        if (requestCode == CAMERA_PERMISSION_CODE &&
            grantResults.isNotEmpty() &&
            grantResults[0] == PackageManager.PERMISSION_GRANTED) {
            setupSession()
        }
    }

    override fun onResume() {
        super.onResume()
        glView.onResume()
        session?.resume()
    }

    override fun onPause() {
        super.onPause()
        session?.pause()
        glView.onPause()
    }

    override fun onDestroy() {
        super.onDestroy()
        session?.close()
    }
}

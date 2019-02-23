package org.opencv.android;

import java.lang.ref.WeakReference;
import java.nio.ByteBuffer;
import java.util.List;

import android.content.Context;
import android.graphics.ImageFormat;
import android.graphics.SurfaceTexture;
import android.hardware.Camera;
import android.hardware.Camera.PreviewCallback;
import android.hardware.usb.UsbDevice;
import android.os.Build;
import android.os.Handler;
import android.os.Looper;
import android.os.Message;
import android.util.AttributeSet;
import android.util.Log;
import android.view.Surface;
import android.view.View;
import android.view.ViewGroup.LayoutParams;

import com.android.jackapp.jusbcam.RequestUsbPermission;
import com.android.jackapp.jusbcam.usb.IFrameCallback;
import com.android.jackapp.jusbcam.usb.USBMonitor;
import com.android.jackapp.jusbcam.usb.UVCCamera;
import com.android.jackapp.jusbcam.widget.CameraViewInterface;

import org.opencv.BuildConfig;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

/**
 * This class is an implementation of the Bridge View between OpenCV and Java Camera.
 * This class relays on the functionality available in base class and only implements
 * required functions:
 * connectCamera - opens Java camera and sets the PreviewCallback to be delivered.
 * disconnectCamera - closes the camera and stops preview.
 * When frame is delivered via callback from Camera - it processed via OpenCV to be
 * converted to RGBA32 and then passed to the external callback for modifications if required.
 */
public class JavaCameraView extends CameraBridgeViewBase implements PreviewCallback {

    private static final int MAGIC_TEXTURE_ID = 10;
    private static final String TAG = "JavaCameraView";
    private static Context mContext;//cj

    private byte mBuffer[];
    private Mat[] mFrameChain;
    private int mChainIdx = 0;
    private Thread mThread;
    private boolean mStopThread;

//    protected Camera mCamera;
    protected JavaCameraFrame[] mCameraFrame;
    private SurfaceTexture mSurfaceTexture;
    private int mPreviewFormat = ImageFormat.NV21;
    private USBMonitor mUSBMonitor = null;
    private USBMonitor.OnDeviceConnectListener mOnDeviceConnectListener = new USBMonitor.OnDeviceConnectListener() {
        @Override
        public void onAttach(UsbDevice device) {

        }

        @Override
        public void onDettach(UsbDevice device) {

        }

        @Override
        public void onConnect(UsbDevice device, USBMonitor.UsbControlBlock ctrlBlock, boolean createNew) {
            mHandler.openCamera(ctrlBlock);
            //preview
                mSurfaceTexture = new SurfaceTexture(MAGIC_TEXTURE_ID);

                if (mSurface != null) {
                    mSurface.release();
                }
                mSurface = new Surface(mSurfaceTexture);
                mHandler.startPreview(mSurface);
                initializeCamera(640, 480);

                mCameraFrameReady = false;

                mStopThread = false;
                mThread = new Thread(new CameraWorker());
                mThread.start();
        }

        @Override
        public void onDisconnect(UsbDevice device, USBMonitor.UsbControlBlock ctrlBlock) {
            if (mHandler != null) {
                mHandler.closeCamera();
            }
        }

        @Override
        public void onCancel() {

        }
    };

    private CameraHandler mHandler;
    private Surface mSurface;

    public static class JavaCameraSizeAccessor implements ListItemAccessor {

        @Override
        public int getWidth(Object obj) {
            Camera.Size size = (Camera.Size) obj;
            return size.width;
        }

        @Override
        public int getHeight(Object obj) {
            Camera.Size size = (Camera.Size) obj;
            return size.height;
        }
    }

    public JavaCameraView(Context context, int cameraId) {
        super(context, cameraId);
        jInit(context);
    }

    public JavaCameraView(Context context, AttributeSet attrs) {
        super(context, attrs);
        jInit(context);
    }

    private void jInit(Context context) {
        mContext = context;
        if(mUSBMonitor == null) {
            mUSBMonitor = new USBMonitor(mContext, mOnDeviceConnectListener);
            mUSBMonitor.register();
        }
         if(mHandler == null) {
             CameraHandler ch = new CameraHandler();
             mHandler = ch.createHandler(mContext);
         }
    }

    private void jExit() {
        if (mUSBMonitor != null) {
            mUSBMonitor.unregister();
            mUSBMonitor.destroy();
            mUSBMonitor = null;
        }
        if(mHandler != null) {
            mHandler.closeCamera();
        }
    }

    protected boolean initializeCamera(int width, int height) {
        Log.d(TAG, "Initialize java camera");
        boolean result = true;
        synchronized (this) {
//            mCamera = null;

            Log.e("cjtest","init camera ");


/*

            if (mCamera == null)
                return false;
*/

            /* Now set camera parameters */
            try {
/*                Camera.Parameters params = mCamera.getParameters();
                Log.d(TAG, "getSupportedPreviewSizes()");
                List<android.hardware.Camera.Size> sizes = params.getSupportedPreviewSizes();

                if (sizes != null) {*/
                    /* Select the size that fits surface considering maximum size allowed */
//                    Size frameSize = calculateCameraFrameSize(sizes, new JavaCameraSizeAccessor(), width, height);

                    /* Image format NV21 causes issues in the Android emulators */
/*                    if (Build.FINGERPRINT.startsWith("generic")
                            || Build.FINGERPRINT.startsWith("unknown")
                            || Build.MODEL.contains("google_sdk")
                            || Build.MODEL.contains("Emulator")
                            || Build.MODEL.contains("Android SDK built for x86")
                            || Build.MANUFACTURER.contains("Genymotion")
                            || (Build.BRAND.startsWith("generic") && Build.DEVICE.startsWith("generic"))
                            || "google_sdk".equals(Build.PRODUCT))
                        params.setPreviewFormat(ImageFormat.YV12);  // "generic" or "android" = android emulator
                    else
                        params.setPreviewFormat(ImageFormat.NV21);

                    mPreviewFormat = params.getPreviewFormat();

                    Log.d(TAG, "Set preview size to " + Integer.valueOf((int)frameSize.width) + "x" + Integer.valueOf((int)frameSize.height));
                    params.setPreviewSize((int)frameSize.width, (int)frameSize.height);*/

/*
                    if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.ICE_CREAM_SANDWICH && !android.os.Build.MODEL.equals("GT-I9100"))
                        params.setRecordingHint(true);

                    List<String> FocusModes = params.getSupportedFocusModes();
                    if (FocusModes != null && FocusModes.contains(Camera.Parameters.FOCUS_MODE_CONTINUOUS_VIDEO))
                    {
                        params.setFocusMode(Camera.Parameters.FOCUS_MODE_CONTINUOUS_VIDEO);
                    }

                    mCamera.setParameters(params);
                    params = mCamera.getParameters();
*/

                    mFrameWidth = 640;//params.getPreviewSize().width;
                    mFrameHeight = 480;//params.getPreviewSize().height;

                    if ((getLayoutParams().width == LayoutParams.MATCH_PARENT) && (getLayoutParams().height == LayoutParams.MATCH_PARENT))
                        mScale = Math.min(((float)height)/mFrameHeight, ((float)width)/mFrameWidth);
                    else
                        mScale = 0;

                    if (mFpsMeter != null) {
                        mFpsMeter.setResolution(mFrameWidth, mFrameHeight);
                    }

                    int size = mFrameWidth * mFrameHeight;
                    //size  = size * ImageFormat.getBitsPerPixel(params.getPreviewFormat()) / 8;
                    mBuffer = new byte[/*size*/size*12/8];

//                    mCamera.addCallbackBuffer(mBuffer);
//                    mCamera.setPreviewCallbackWithBuffer(this);

                    mFrameChain = new Mat[2];
                    mFrameChain[0] = new Mat(mFrameHeight + (mFrameHeight/2), mFrameWidth, CvType.CV_8UC1);
                    mFrameChain[1] = new Mat(mFrameHeight + (mFrameHeight/2), mFrameWidth, CvType.CV_8UC1);

                    AllocateCache();

                    mCameraFrame = new JavaCameraFrame[2];
                    mCameraFrame[0] = new JavaCameraFrame(mFrameChain[0], mFrameWidth, mFrameHeight);
                    mCameraFrame[1] = new JavaCameraFrame(mFrameChain[1], mFrameWidth, mFrameHeight);

     /*               if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.HONEYCOMB) {
                        mSurfaceTexture = new SurfaceTexture(MAGIC_TEXTURE_ID);
                        mCamera.setPreviewTexture(mSurfaceTexture);
                    } else
                       mCamera.setPreviewDisplay(null);

                    *//* Finally we are ready to start the preview *//*
                    Log.d(TAG, "startPreview");
                    mCamera.startPreview();*/
/*                }
                else
                    result = false;*/
                result = true;
            } catch (Exception e) {
                result = false;
                e.printStackTrace();
            }
        }

        return result;
    }

    protected void releaseCamera() {
        synchronized (this) {
            jExit();
/*            if (mCamera != null) {
                mCamera.stopPreview();
                mCamera.setPreviewCallback(null);

                mCamera.release();
            }
            mCamera = null;*/
            if (mFrameChain != null) {
                mFrameChain[0].release();
                mFrameChain[1].release();
            }
            if (mCameraFrame != null) {
                mCameraFrame[0].release();
                mCameraFrame[1].release();
            }
        }
    }

    private boolean mCameraFrameReady = false;

    @Override
    protected boolean connectCamera(int width, int height) {

        /* 1. We need to instantiate camera
         * 2. We need to start thread which will be getting frames
         */
        /* First step - initialize camera connection */
        Log.d(TAG, "Connecting to camera");
        jInit(mContext);

        RequestUsbPermission reqPermission = new RequestUsbPermission(mContext, mUSBMonitor);
        reqPermission.requestPermission(reqPermission.getDevice());

        return true;
    }

    @Override
    protected void disconnectCamera() {
        /* 1. We need to stop thread which updating the frames
         * 2. Stop camera and release it
         */
        Log.d(TAG, "Disconnecting from camera");
        try {
            mStopThread = true;
            Log.d(TAG, "Notify thread");
            synchronized (this) {
                this.notify();
            }
            Log.d(TAG, "Waiting for thread");
            if (mThread != null)
                mThread.join();
        } catch (InterruptedException e) {
            e.printStackTrace();
        } finally {
            mThread =  null;
        }

        /* Now release camera */
        releaseCamera();

        mCameraFrameReady = false;
    }

    @Override
    public void onPreviewFrame(byte[] frame, Camera arg1) {
        if (BuildConfig.DEBUG)
            Log.d(TAG, "Preview Frame received. Frame size: " + frame.length);
        synchronized (this) {
            mFrameChain[mChainIdx].put(0, 0, frame);
            mCameraFrameReady = true;
            this.notify();
        }
/*        if (mCamera != null)
            mCamera.addCallbackBuffer(mBuffer);*/
    }

    private void jPreviceFrame(final ByteBuffer buffer) {
        byte[] frame = new byte[buffer.remaining()];
        buffer.get(frame,0,frame.length);

        synchronized (this) {
            mFrameChain[mChainIdx].put(0, 0, frame);
            mCameraFrameReady = true;
            this.notify();
        }
    }
    private class JavaCameraFrame implements CvCameraViewFrame {
        @Override
        public Mat gray() {
            return mYuvFrameData.submat(0, mHeight, 0, mWidth);
        }

        @Override
        public Mat rgba() {
            if (mPreviewFormat == ImageFormat.NV21)
                Imgproc.cvtColor(mYuvFrameData, mRgba, Imgproc.COLOR_YUV2RGBA_NV21, 4);
            else if (mPreviewFormat == ImageFormat.YV12)
                Imgproc.cvtColor(mYuvFrameData, mRgba, Imgproc.COLOR_YUV2RGB_I420, 4);  // COLOR_YUV2RGBA_YV12 produces inverted colors
            else
                throw new IllegalArgumentException("Preview Format can be NV21 or YV12");

            return mRgba;
        }

        public JavaCameraFrame(Mat Yuv420sp, int width, int height) {
            super();
            mWidth = width;
            mHeight = height;
            mYuvFrameData = Yuv420sp;
            mRgba = new Mat();
        }

        public void release() {
            mRgba.release();
        }

        private Mat mYuvFrameData;
        private Mat mRgba;
        private int mWidth;
        private int mHeight;
    };

    private class CameraWorker implements Runnable {

        @Override
        public void run() {
            do {
                boolean hasFrame = false;
                synchronized (JavaCameraView.this) {
                    try {
                        while (!mCameraFrameReady && !mStopThread) {
                            JavaCameraView.this.wait();
                        }
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    if (mCameraFrameReady)
                    {
                        mChainIdx = 1 - mChainIdx;
                        mCameraFrameReady = false;
                        hasFrame = true;
                    }
                }

                if (!mStopThread && hasFrame) {
                    if (!mFrameChain[1 - mChainIdx].empty())
                        deliverAndDrawFrame(mCameraFrame[1 - mChainIdx]);
                }
            } while (!mStopThread);
            Log.d(TAG, "Finish processing thread");
        }
    }



    private final class CameraHandler extends Handler {
        private static final int MSG_OPEN = 0;
        private static final int MSG_CLOSE = 1;
        private static final int MSG_PREVIEW_START = 2;
        private static final int MSG_PREVIEW_STOP = 3;
        private static final int MSG_CAPTURE_STILL = 4;
        private static final int MSG_CAPTURE_START = 5;
        private static final int MSG_CAPTURE_STOP = 6;
        private static final int MSG_MEDIA_UPDATE = 7;
        private static final int MSG_RELEASE = 9;

        private WeakReference<CameraThread> mWeakThread = null;

        public final CameraHandler createHandler(final Context parent) {
            final CameraThread thread = new CameraThread(parent);
            thread.start();
            return thread.getHandler();
        }

        private CameraHandler() {
        }

        private CameraHandler(final CameraThread thread) {
            mWeakThread = new WeakReference<CameraThread>(thread);
        }

        public boolean isCameraOpened() {
            final CameraThread thread = mWeakThread.get();
            return thread != null ? thread.isCameraOpened() : false;
        }

        public void openCamera(final USBMonitor.UsbControlBlock ctrlBlock) {
            sendMessage(obtainMessage(MSG_OPEN, ctrlBlock));
        }

        public void closeCamera() {
            stopPreview();
            sendEmptyMessage(MSG_CLOSE);
        }

        public void startPreview(final Surface sureface) {
            if (sureface != null)
                sendMessage(obtainMessage(MSG_PREVIEW_START, sureface));
        }

        public void stopPreview() {
            final CameraThread thread = mWeakThread.get();
            if (thread == null) return;
            synchronized (thread.mSync) {
                sendEmptyMessage(MSG_PREVIEW_STOP);
                // wait for actually preview stopped to avoid releasing Surface/SurfaceTexture
                // while preview is still running.
                // therefore this method will take a time to execute
                try {
                    thread.mSync.wait();
                } catch (final InterruptedException e) {
                }
            }
        }


        @Override
        public void handleMessage(final Message msg) {
            final CameraThread thread = mWeakThread.get();
            if (thread == null) return;
            switch (msg.what) {
                case MSG_OPEN:
                    thread.handleOpen((USBMonitor.UsbControlBlock)msg.obj);
                    break;
                case MSG_CLOSE:
                    thread.handleClose();
                    break;
                case MSG_PREVIEW_START:
                    thread.handleStartPreview((Surface)msg.obj);
                    break;
                case MSG_PREVIEW_STOP:
                    thread.handleStopPreview();
                    break;
                case MSG_RELEASE:
                    thread.handleRelease();
                    break;
                default:
                    throw new RuntimeException("unsupported message:what=" + msg.what);
            }
        }

    private final class CameraThread extends Thread {
        private static final String TAG_THREAD = "CameraThread";
        private static final boolean isAudioRecord = false;
        private final Object mSync = new Object();
        private final WeakReference<Context> mWeakParent;
        private boolean mIsRecording;

        private CameraHandler mHandler;
        /**
         * for accessing UVC camera
         */
        private UVCCamera mUVCCamera;


        private CameraThread(final Context parent) {
            super("CameraThread");
            mWeakParent = new WeakReference<Context>(parent);
        }

        @Override
        protected void finalize() throws Throwable {
            Log.i(TAG, "CameraThread#finalize");
            super.finalize();
        }

        public CameraHandler getHandler() {
            synchronized (mSync) {
                if (mHandler == null)
                    try {
                        mSync.wait();
                    } catch (final InterruptedException e) {
                    }
            }
            return mHandler;
        }

        public boolean isCameraOpened() {
            return mUVCCamera != null;
        }


        public void handleOpen(final USBMonitor.UsbControlBlock ctrlBlock) {
            handleClose();
            mUVCCamera = new UVCCamera();
            mUVCCamera.open(ctrlBlock);
        }

        public void handleClose() {
            if (mUVCCamera != null) {
                mUVCCamera.stopPreview();
                mUVCCamera.destroy();
                mUVCCamera = null;
            }
        }

        public void handleStartPreview(final Surface surface) {

            if (mUVCCamera == null) return;
            try {
                mUVCCamera.setPreviewSize(640, 480, UVCCamera.FRAME_FORMAT_MJPEG);
            } catch (final IllegalArgumentException e) {
                try {
                    // fallback to YUV mode
                    mUVCCamera.setPreviewSize(640, 480, UVCCamera.DEFAULT_PREVIEW_MODE);
                } catch (final IllegalArgumentException e1) {
                    handleClose();
                }
            }
            if (mUVCCamera != null) {
                mUVCCamera.setPreviewDisplay(surface);
                mUVCCamera.startPreview();
                mUVCCamera.setFrameCallback(mIFrameCallback,UVCCamera.PIXEL_FORMAT_NV21);
            }
        }

        public void handleStopPreview() {
            if (mUVCCamera != null) {
                mUVCCamera.stopPreview();
            }
            synchronized (mSync) {
                mSync.notifyAll();
            }
        }



        public void handleRelease() {
            handleClose();
            if (!mIsRecording)
                Looper.myLooper().quit();
        }

        private final IFrameCallback mIFrameCallback = new IFrameCallback() {
            @Override
            public void onFrame(final ByteBuffer frame) {
/*                if (mVideoEncoder != null) {
                    mVideoEncoder.frameAvailableSoon();
                    mVideoEncoder.encode(frame);
                }*/
                jPreviceFrame(frame);
            }
        };

        @Override
        public void run() {
            Looper.prepare();
            synchronized (mSync) {
                mHandler = new CameraHandler(this);
                mSync.notifyAll();
            }
            Looper.loop();
            synchronized (mSync) {
                mHandler = null;
                mSync.notifyAll();
            }
        }
    }
}
}

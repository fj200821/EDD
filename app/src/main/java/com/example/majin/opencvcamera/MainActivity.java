package com.example.majin.opencvcamera;
import android.app.Activity;
import android.app.admin.DevicePolicyManager;
import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.graphics.PixelFormat;
import android.os.Bundle;
import android.util.Log;
import android.view.Gravity;
import android.view.LayoutInflater;
import android.view.MotionEvent;
import android.view.View;
import android.view.Window;
import android.view.WindowManager;
import android.widget.FrameLayout;
import android.widget.Toast;
import android.view.KeyEvent;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import static org.opencv.imgproc.Imgproc.CHAIN_APPROX_NONE;
import static org.opencv.imgproc.Imgproc.COLOR_BGR2GRAY;
import static org.opencv.imgproc.Imgproc.MORPH_RECT;
import static org.opencv.imgproc.Imgproc.RETR_EXTERNAL;
import static org.opencv.imgproc.Imgproc.THRESH_BINARY;
import static org.opencv.imgproc.Imgproc.boundingRect;
import static org.opencv.imgproc.Imgproc.dilate;
import static org.opencv.imgproc.Imgproc.erode;
import static org.opencv.imgproc.Imgproc.getStructuringElement;

public class MainActivity extends Activity implements CameraBridgeViewBase.CvCameraViewListener2 {
    //以下全局标识
    private String TAG = "ElevatorDetectorModel";
    private String logtag = "EDDPhone";
    private static final String KEY_ACTION = "com.newhaisiwei.keycode";
    private static final String EXTRA_KEY_VALUE = "KeyCode";

    //OpenCV的相机接口
    private CameraBridgeViewBase mCVCamera;

    //以下为参数定义
    private int YOffset,SubMatHight,OpenMin,OpenMax,CloseMin,CloseMax,FrameOut;

    //以下全局变量
    private Mat mRgba,SubL,SubR;
    private int swid,shei,nLR,nClose;
    private int[][] LR= {{0,0,0},{0,0,0}};

    private int nFrmNum,nFrmSync,CloseFrame;
    private boolean runflag,ifClosed;

    //以下IO类及变量
    private LogToFile FileLogs;
    private String LogStr;
    private CommandExecution CmdExec;
    private CommandExecution.CommandResult CmdResult;
    //定义浮动窗口布局
    FrameLayout mFloatLayout;
    WindowManager.LayoutParams wmParams;
    //创建浮动窗口设置布局参数的对象
    WindowManager mWindowManager;


    DevicePolicyManager policyManager;
    ComponentName adminReceiver;
    /**
     * 通过OpenCV管理Android服务，异步初始化OpenCV
     */
    BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(this) {
        @Override
        public void onManagerConnected(int status) {
            switch (status) {
                case LoaderCallbackInterface.SUCCESS:
                    Log.i(TAG, "OpenCV loaded successfully");
                    mCVCamera.enableView();
                    break;
                default:
                    break;
            }
        }
    };
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        // 无title
        requestWindowFeature(Window.FEATURE_NO_TITLE);
        // 全屏
        getWindow().setFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN,
                WindowManager.LayoutParams.FLAG_FULLSCREEN);
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        createFloatView();
        FileLogs.init(this);
        //赋值摄像头对象
        //mCVCamera = (CameraBridgeViewBase) findViewById(R.id.camera_view);
        mCVCamera.setCvCameraViewListener(this);
        // 打开USB摄像头 ID=0
        mCVCamera.setCameraIndex(-1);

        //mPowerManager = (PowerManager) getSystemService(POWER_SERVICE);
        policyManager = (DevicePolicyManager) MainActivity.this.getSystemService(Context.DEVICE_POLICY_SERVICE);

        Intent intent = new Intent(DevicePolicyManager.ACTION_ADD_DEVICE_ADMIN);
        adminReceiver = new ComponentName(MainActivity.this, ScreenOffAdminReceiver.class);
        intent.putExtra(DevicePolicyManager.EXTRA_DEVICE_ADMIN,  adminReceiver);
        intent.putExtra(DevicePolicyManager.EXTRA_ADD_EXPLANATION,"开启后就可以使用锁屏功能了...");//显示位置见图二

        startActivityForResult(intent, 0);
    }
    @Override
    public void onResume() {
        super.onResume();
        if (!OpenCVLoader.initDebug()) {
            Log.d(TAG, "OpenCV library not found!");
        } else {
            Log.d(TAG, "OpenCV library found inside package. Using it!");
            mLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }

    }

    @Override
    public void onDestroy() {
        runflag = false;
        if(mCVCamera!=null){
            mCVCamera.disableView();
        }
        super.onDestroy();
    }

    private void InitPara(){

        //UI中配置文件中的参数这里赋值
        YOffset = 2;        //检测区域距离Top的偏移
        SubMatHight = 60;   //检测区域的高度
        OpenMin = 5;
        OpenMax = 100;
        CloseMin = -5;
        CloseMax = -100;
        FrameOut = 50;
        
        mRgba = new Mat(shei, swid, CvType.CV_8UC4); //原始RGBA四通道图像（携带Alpha透明度信息的PNG图像）
        SubL = new Mat();
        SubR = new Mat();
        nLR = 1;
        nClose = 0;
        nFrmSync = 0;//初始化帧数计数器及同步计数器
        CloseFrame = 0;
        runflag = true;
        ifClosed = true;
    }
    //切换光机状态，输入的ON_OFF为true时，光机若关闭，则打开，反之亦然。
    private boolean SwitchLight(boolean ON_OFF){
        CmdResult = CmdExec.execCommand("cat /sys/class/leds/lcd-backlight/brightness",false);
        if(CmdResult.result>=0) {
            if (CmdResult.successMsg.equals("0")^ON_OFF) OnOffScreen();
            return true;
        }
        else{
            return false;
        }
    }

    private int DiffMat(Mat NowSub,Mat LastSub,boolean ifL){
        Mat mDiff = new Mat();
        Mat kernel_erode;
        Mat kernel_dilate;
        int DiffOffset;
        Core.absdiff(NowSub, LastSub, mDiff);
        Imgproc.threshold(mDiff, mDiff, 50, 255, THRESH_BINARY);
        kernel_erode = getStructuringElement(MORPH_RECT, new Size(3, 3));
        kernel_dilate = getStructuringElement(MORPH_RECT, new Size(15, 15));
        erode(mDiff, mDiff, kernel_erode);
        dilate(mDiff, mDiff, kernel_dilate);
        kernel_erode.release();
        kernel_dilate.release();
        List<MatOfPoint> contours=new ArrayList<>();
        Imgproc.findContours(mDiff, contours, new Mat(), RETR_EXTERNAL, CHAIN_APPROX_NONE);
        Rect[] boundRect = new Rect[contours.size()];
        DiffOffset=swid/2;
        for (int i = 0; i < contours.size(); i++)
        {
            boundRect[i] = boundingRect(contours.get(i));
            if(boundRect[i].height<SubMatHight/2) continue;
            if(ifL){
                if(DiffOffset > swid/2-boundRect[i].x-boundRect[i].width) DiffOffset = swid/2-boundRect[i].x-boundRect[i].width;
            }
            else{
                if(DiffOffset > boundRect[i].x - swid/2) DiffOffset = boundRect[i].x - swid/2;
            }
        }
        mDiff.release();
        return DiffOffset;
    }

    private void CompareFrame(Mat NowMat){
        Mat SubA;
        Mat SubB;
        int LDiff,RDiff;
        SubA = NowMat.submat(new Rect(0,YOffset,swid/2,SubMatHight));
        SubB = NowMat.submat(new Rect(swid/2,YOffset,swid/2,SubMatHight));
        Imgproc.cvtColor(SubA,SubA,COLOR_BGR2GRAY);
        Imgproc.cvtColor(SubB,SubB,COLOR_BGR2GRAY);
        LDiff = DiffMat(SubA,SubL,true);
        RDiff = DiffMat(SubB,SubR,false);
        if(LDiff<swid/2){
            LR[0][nLR%3] = LDiff;
            SubA.copyTo(SubL);
        }
        else{
            LR[0][nLR%3] = LR[0][(nLR-1)%3];
        }
        if(RDiff<swid/2){
            LR[1][nLR%3] = RDiff;
            SubB.copyTo(SubR);
        }
        else{
            LR[1][nLR%3] = LR[0][(nLR-1)%3];
        }
        nLR++;
        if(nLR>=(Integer.MAX_VALUE)) nLR = 1;
        SubA.release();
        SubB.release();
    }
    private boolean Analy(int NowFrame){
        boolean result = ifClosed;
        int LO,RO,LS,RS;
        LS = (LR[0][nLR%3]-LR[0][(nLR-2)%3])/2;
        RS = (LR[1][nLR%3]-LR[1][(nLR-2)%3])/2;
        if(LS>OpenMin&&LS<OpenMax&&RS>OpenMin&&RS<OpenMax){
            nClose = 0;
            result = false;
        }
        else if(LS<CloseMin&&LS>CloseMax&&RS<CloseMin&&RS>CloseMax){
            CloseFrame = NowFrame;
            nClose++;
        }
        if(nClose>2){
            LO = LR[0][nLR%3];
            RO = LR[1][nLR%3];
            if(LO==0&&RO==0&&LS==0&&RS==0){
                nClose = 0;
                result = true;
            }
        }
        if(nClose>0&&(NowFrame - CloseFrame)>FrameOut) nClose = 0;      //超时 关门中状态失效，nClose清零
        return result;
    }

    private void varrun(){
        while(runflag) {
            try {
                while((nFrmSync==nFrmNum)&&runflag){//等待新的帧存入缓冲区
                    Thread.sleep(1);
                }
                CompareFrame(mRgba);
                if(Analy(nFrmNum)^ifClosed){    //计算当前位置速度，判定是否与全局状态变量一致
                    ifClosed = !ifClosed;           //不一致则切换全局状态变量
                    SwitchLight(ifClosed);          //并将状态传递给光机
                }
                nFrmSync = nFrmNum;//同步
            }
            catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        LogStr = "线程V退出";
        FileLogs.i(logtag,LogStr);
    }
/*
    private byte ComparePoint(Mat MatA,Mat MatB){
        byte NowPoint[][][] = new byte[2][4][1];
        Imgproc.cvtColor(MatA, MatA, COLOR_BGR2GRAY);
        Imgproc.cvtColor(MatB, MatB, COLOR_BGR2GRAY);
        MatA.get(YOffset,swid/2-LMax,NowPoint[0][0]);
        MatA.get(YOffset,swid*(MiddleValue-1)/(MiddleValue*2),NowPoint[0][1]);
        MatA.get(YOffset,swid*(MiddleValue+1)/(MiddleValue*2),NowPoint[0][2]);
        MatA.get(YOffset,swid/2+RMax-1,NowPoint[0][3]);
        MatB.get(YOffset,swid/2-LMax,NowPoint[1][0]);
        MatB.get(YOffset,swid*(MiddleValue-1)/(MiddleValue*2),NowPoint[1][1]);
        MatB.get(YOffset,swid*(MiddleValue+1)/(MiddleValue*2),NowPoint[1][2]);
        MatB.get(YOffset,swid/2+RMax-1,NowPoint[1][3]);
        byte result = 0;
        for(int i=0;i<4;i++){
            if(NowPoint[0][i][0]!=NowPoint[1][i][0]) result += Math.pow(2,i);
        }
        return result;
    }
*/
    @Override
    public void onCameraViewStarted(int width, int height) {
        //初始化摄像头分辨率
        if(width==0||height==0) {
            LogStr = "UVC摄像头，固定分辨率640*480";
            width = 640;//USB摄像头需要自定义分辨率（画面宽高）
            height = 480;
        }
        else LogStr = String.format(Locale.CHINA,"原生摄像头，分辨率%d*%d",width,height);
        FileLogs.i(logtag,LogStr);

        swid = width;//原生摄像头可以自动获取分辨率（画面宽高）
        shei = height;
        //初始化变量及参数
        InitPara();
        //启动逐帧分析线程
        Thread threadvar = new Thread("ThreadVar") {
            public void run(){
                varrun();
            }
        };
        threadvar.start();
    }

    @Override
    public void onCameraViewStopped() {
        mRgba.release();
    }

    //private PowerManager mPowerManager;
    //private PowerManager.WakeLock mWakeLock;

    private void createFloatView()
    {
        wmParams = new WindowManager.LayoutParams();
        //获取WindowManagerImpl.CompatModeWrapper
        //mWindowManager = (WindowManager)getApplication().getSystemService(getApplication().WINDOW_SERVICE);
        mWindowManager = (WindowManager)getApplication().getSystemService(Context.WINDOW_SERVICE);
        //设置window type
        wmParams.type = WindowManager.LayoutParams.TYPE_PHONE;
        //设置图片格式，效果为背景透明
        wmParams.format = PixelFormat.RGBA_8888;
        //设置浮动窗口不可聚焦（实现操作除浮动窗口外的其他可见窗口的操作）
        wmParams.flags = WindowManager.LayoutParams.FLAG_NOT_FOCUSABLE;

        //调整悬浮窗显示的停靠位置为左侧置顶
        //wmParams.gravity = Gravity.LEFT | Gravity.TOP;
        wmParams.gravity = Gravity.START | Gravity.TOP;

        // 以屏幕左上角为原点，设置x、y初始值
        wmParams.x = 300;
        wmParams.y = 5;

        // 设置悬浮窗口长宽数据
        wmParams.width = 120;
        wmParams.height = 160;

        LayoutInflater inflater = LayoutInflater.from(getApplication());
        //获取浮动窗口视图所在布局
        mFloatLayout = (FrameLayout) inflater.inflate(R.layout.activity_main, null);
        //添加mFloatLayout
        mWindowManager.addView(mFloatLayout, wmParams);
        mCVCamera = (CameraBridgeViewBase)  mFloatLayout.findViewById(R.id.camera_view);
        mFloatLayout.measure(View.MeasureSpec.makeMeasureSpec(0,
                View.MeasureSpec.UNSPECIFIED), View.MeasureSpec
                .makeMeasureSpec(0, View.MeasureSpec.UNSPECIFIED));
        //设置监听浮动窗口的触摸移动
        mFloatLayout.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                // TODO Auto-generated method stub
                wmParams.x = (int) event.getRawX() - mFloatLayout.getMeasuredWidth()/2;
                wmParams.y = (int) event.getRawY() - mFloatLayout.getMeasuredHeight()/2;
                mWindowManager.updateViewLayout(mFloatLayout, wmParams);
                return false;
            }
        });
    }

    public void OnOffScreen() {
        Intent intent = new Intent(KEY_ACTION);
        intent.putExtra(EXTRA_KEY_VALUE, KeyEvent.KEYCODE_POWER);
        sendBroadcast(intent);
    }

    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        super.onActivityResult(requestCode, resultCode, data);
        isOpen();
    }

    /**
     * 检测用户是否开启了超级管理员
     */
    private void isOpen() {
        if(policyManager.isAdminActive(adminReceiver)){//判断超级管理员是否激活

            Toast.makeText(MainActivity.this,"设备已被激活",
                    Toast.LENGTH_LONG).show();

        }else{

            Toast.makeText(MainActivity.this,"设备没有被激活",
                    Toast.LENGTH_LONG).show();

        }
    }

    /**
     * 图像处理都写在此处
     */
    @Override
    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
        mRgba = inputFrame.rgba();//本方法内部不对帧进行重绘则可以省掉复制过程。
        if(nFrmNum>=(Integer.MAX_VALUE)) nFrmNum = 0;//按每秒30帧算，Integer.MAX_VALUE=2147483647，折合828天半。超过这个运行时间，计数器归零，且不影响循环判断。
        nFrmNum++;
        return mRgba;//此处必须返回与控件相同分辨率的mat，否则无法显示。
    }
}
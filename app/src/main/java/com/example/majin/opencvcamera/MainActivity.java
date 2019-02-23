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
    private String TAG = "ElevatorDetectorModel";
    private String logtag = "EDDPhone";

    private static final String KEY_ACTION = "com.newhaisiwei.keycode";
    private static final String EXTRA_KEY_VALUE = "KeyCode";
    //OpenCV的相机接口
    private CameraBridgeViewBase mCVCamera;
    //缓存相机每帧输入的数据
    private Mat mRgba,mLast,mOpen,mClose,mFrame,m_diff;
    private int swid,shei;

    //以下为参数定义
    private int FrameNum,YOffset,SubMatHight;
    //以上为参数定义

    private int nFrmNum,nFrmSync,nProcNum,nGetPoint;
    private int TotalDiff,TotalArea,NowState,SyncState;
    private int Lwidth,Rwidth,LtoR,MiddleValue,LMax,RMax,VarMin,PointMin;
    private int DiffBuffer[][];
    private int Points[][][];
    private boolean runflag,moveflag,getclose,ifclose,lightstatus,iflight;
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
        FrameNum = 10;       //采样的帧数，同时会根据此值定义缓存数量。平衡性能和效率的参数。
        YOffset = 2;        //检测区域距离Top的偏移
        SubMatHight = 60;   //检测区域的高度

        mRgba = new Mat(shei, swid, CvType.CV_8UC4); //原始RGBA四通道图像（携带Alpha透明度信息的PNG图像）
        mLast = new Mat();
        mOpen = new Mat();
        mClose = new Mat();
        mFrame = new Mat();
        m_diff = new Mat();
        DiffBuffer = new int[FrameNum][4];
        for(int i=0;i<FrameNum;i++){
            DiffBuffer[i][0] = 2;
            DiffBuffer[i][1] = 0;
            DiffBuffer[i][2] = 0;
            DiffBuffer[i][3] = 0;
        }

        Points = new int[3][2][5];                //每扇门四个极限点,一个极限长，两扇门，三组作为基准对比，误差全小于Pointvar则为准备好。
        for(int i=0;i<30;i++){
            Points[i/10][(i/5)%2][i%5] = 0;
        }
        TotalDiff = FrameNum*2;
        TotalArea = 0;
        MiddleValue = 40;
        VarMin = 50;
        PointMin = 30;
        LMax=RMax=swid/2;
        nFrmNum = nFrmSync = nGetPoint = 0;//初始化帧数计数器及同步计数器
        nProcNum = 1;//初始化帧处理计数器
        NowState = 0;
        SyncState = 0;
        runflag = true;
        moveflag = true;
        getclose  = false;
        ifclose = true;
        //查询光机状态
        CmdResult = CmdExec.execCommand("cat /sys/class/leds/lcd-backlight/brightness",false);
        if(CmdResult.result>=0) {
            lightstatus = true;
            if (CmdResult.successMsg == "0") {
                iflight = false;
            }
            else {
                iflight = true;
            }
        }
        else{
            lightstatus = false;
            iflight = true;
        }
        //LSD = Imgproc.createLineSegmentDetector();
    }
    private  Mat ProcessFrame(Mat MatA, Mat MatB){
        Mat SubA;
        Mat SubB;
        Mat GrayA = new Mat();
        Mat GrayB = new Mat();
        Mat mDiff = new Mat();
        Mat kernel_erode;
        Mat kernel_dilate;

        SubA = MatA.submat(new Rect(0,YOffset,swid,SubMatHight));
        SubB = MatB.submat(new Rect(0,YOffset,swid,SubMatHight));
        Imgproc.cvtColor(SubA,GrayA,COLOR_BGR2GRAY);
        Imgproc.cvtColor(SubB,GrayB,COLOR_BGR2GRAY);
        Core.absdiff(GrayA, GrayB, mDiff);
        Imgproc.threshold(mDiff, mDiff, 50, 255, THRESH_BINARY);
        kernel_erode = getStructuringElement(MORPH_RECT, new Size(3, 3));
        kernel_dilate = getStructuringElement(MORPH_RECT, new Size(15, 15));
        erode(mDiff, mDiff, kernel_erode);
        dilate(mDiff, mDiff, kernel_dilate);
        SubA.release();
        SubB.release();
        GrayA.release();
        GrayB.release();
        kernel_erode.release();
        kernel_dilate.release();
        return mDiff;
    }
    private int AnalyseDiff(Mat MatDiff,int NowFrame){
        List<MatOfPoint> contours=new ArrayList<>();
        Imgproc.findContours(MatDiff, contours, new Mat(), RETR_EXTERNAL, CHAIN_APPROX_NONE);
        Rect[] boundRect = new Rect[contours.size()];
        int NowDiff = 0;
        int NowArea = 0;
        Lwidth=Rwidth=swid/2;
        for (int i = 0; i < contours.size(); i++)
        {
            boundRect[i] = boundingRect(contours.get(i));
            if(boundRect[i].height<SubMatHight/2) continue;
            NowDiff++;                                                                                  //统计多少个不同的块
            NowArea += boundRect[i].width*boundRect[i].height;                                          //统计全部块的面积
            if(boundRect[i].x<swid/2){
                if((boundRect[i].x+boundRect[i].width)>swid/2){
                    Lwidth = Rwidth = 0;
                }
                else{
                    if(Lwidth > swid/2-boundRect[i].x-boundRect[i].width) Lwidth = swid/2-boundRect[i].x-boundRect[i].width;
                }
            }
            else{
                if(Rwidth > boundRect[i].x - swid/2) Rwidth = boundRect[i].x - swid/2;
            }
        }
        if(Lwidth==0&&Rwidth==0&&NowDiff==1) NowDiff = 2;
        TotalDiff = TotalDiff - DiffBuffer[NowFrame%FrameNum][0] + NowDiff; //更新差异块总和（避免for循环资源浪费，采用一减一加）
        DiffBuffer[NowFrame%FrameNum][0] = NowDiff;                     //更新不同数

        TotalArea = TotalArea - DiffBuffer[NowFrame%FrameNum][1] + NowArea;//更新差异面积总和（避免for循环资源浪费，采用一减一加）
        DiffBuffer[NowFrame%FrameNum][1] = NowArea;                     //更新差异面积

        DiffBuffer[NowFrame%FrameNum][2] = Lwidth;                     //更新L无差异距离
        DiffBuffer[NowFrame%FrameNum][3] = Rwidth;                     //更新R无差异距离
        if(LtoR != Lwidth+Rwidth) {
            LtoR = Lwidth+Rwidth;
            LogStr = String.format(Locale.CHINA, "Lwidth:%d  Rwidth:%d，LtoR:%d", Lwidth, Rwidth, LtoR);
            FileLogs.i(logtag, LogStr);
        }
        return NowDiff;
    }

    private void varrun(){
        while(runflag) {
            try {
                if(nFrmNum>1) {//原始两帧装载完成
                    while((nFrmSync==nFrmNum)&&runflag){//等待新的帧存入缓冲区
                        Thread.sleep(1);
                    }
                    m_diff = ProcessFrame(mRgba, mLast);
                    AnalyseDiff(m_diff,nProcNum);
                    if(TotalDiff==0&&moveflag){
                        LogStr = String.format(Locale.CHINA, "当前静止");
                        FileLogs.i(logtag,LogStr);
                        mRgba.copyTo(mLast);
                        moveflag = false;
                        if(NowState==0){
                            NowState = 1;
                            LogStr = getCacheDir().getAbsolutePath()+"/Static.bmp";
                            Imgcodecs.imwrite(LogStr,mFrame);
                            FileLogs.i(logtag,LogStr);
                        }
                        else if(NowState==2){
                            mRgba.copyTo(mClose);
                            GetPoint(mClose,nGetPoint);
                            nGetPoint++;//正常情况下，此值尚未达到MAX，百年已过，不需要设置归零
                            NowState = 4;
                        }
                        else{
                            NowState = 5;
                        }
                    }
                    if(NowState == 5){
                        if(DiffBuffer[(nProcNum-1)%FrameNum][2]==swid/2&&Lwidth<swid/2){
                            LMax = Lwidth;
                            LogStr = String.format(Locale.CHINA, "检测到LMax = %d", LMax);
                            FileLogs.i(logtag, LogStr);
                        }
                        if(DiffBuffer[(nProcNum-1)%FrameNum][3]==swid/2&&Rwidth<swid/2){
                            RMax = Rwidth;
                            LogStr = String.format(Locale.CHINA, "检测到RMax = %d", RMax);
                            FileLogs.i(logtag, LogStr);
                        }
                    }
                    //当出现一半缓冲帧以上的运动被检测到，且左右两边静止长度均低于阀值，则进入运动状态决断
                    if(TotalDiff>FrameNum&&Lwidth<swid/MiddleValue&&Rwidth<swid/MiddleValue&&!moveflag){
                        int DisTotalL = 0;
                        int DisTotalR = 0;
                        int lowdown;
                        //累加缓冲帧中长度降低的次数
                        for(lowdown=1;lowdown<FrameNum;lowdown++){
                            if(DiffBuffer[(nProcNum-lowdown)%FrameNum][2]==swid/2&&DiffBuffer[(nProcNum-lowdown)%FrameNum][3]==swid/2) break;
                            if(DiffBuffer[(nProcNum-lowdown)%FrameNum][2]>DiffBuffer[(nProcNum-lowdown+1)%FrameNum][2]) DisTotalL++;
                            if(DiffBuffer[(nProcNum-lowdown)%FrameNum][3]>DiffBuffer[(nProcNum-lowdown+1)%FrameNum][3]) DisTotalR++;
                        }
                        //当左右两边均降到阀值时，前序缓冲区递减的次数理论上应该为缓冲区的全部，但是如果关门速度过快，可能会略少几帧，但是总不至于3帧内就从全开到全关。
                        if(DisTotalL>2&&DisTotalR>2) {
                            mLast.copyTo(mOpen);
                            NowState = 2;
                            LogStr = String.format(Locale.CHINA, "检测到关门 DL = %d DR = %d Low = %d", DisTotalL, DisTotalR, lowdown);
                            FileLogs.i(logtag, LogStr);

                        }
                        //理论上开门的递减次数应该就为1 从全长降低为0,当没有中缝对齐时，至少有一边的门是从全长一次性降低为0。
                        //else 等同于 if L<3 || R<3 为了容错，0 1 2均判定为中缝迅速变化，即为开门动作
                        else{
                            NowState = 3;
                            LogStr = String.format(Locale.CHINA, "检测到开门 DL = %d DR = %d Low = %d", DisTotalL, DisTotalR, lowdown);
                            FileLogs.i(logtag, LogStr);
                        }
                        moveflag = true;
                    }
                    if(moveflag) mRgba.copyTo(mLast);
                    nProcNum++;//分离处理及同步避免空帧
                    if(nProcNum>=(Integer.MAX_VALUE)) nProcNum = 0;//按每秒30帧算，Integer.MAX_VALUE=2147483647，折合828天半。超过这个运行时间，计数器归零，且不影响循环判断。
                    nFrmSync = nFrmNum;//同步
                }
                else {
                    mRgba.copyTo(mLast);
                    LogStr = "等待帧数据";
                    Thread.sleep(10);
                }
            }
            catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        LogStr = "线程V退出";
        FileLogs.i(logtag,LogStr);
    }

    private void GetPoint(Mat StaticMat,int NowFrame){
        byte NowPoint[][][] = new byte[2][4][1];
        Mat StaticGray = new Mat();
        Imgproc.cvtColor(StaticMat, StaticGray, COLOR_BGR2GRAY);
        StaticGray.get(YOffset,swid/2-LMax,NowPoint[0][0]);
        StaticGray.get(YOffset+SubMatHight,swid/2-LMax,NowPoint[0][1]);
        StaticGray.get(YOffset,swid*(MiddleValue/2-1)/MiddleValue,NowPoint[0][2]);
        StaticGray.get(YOffset+SubMatHight,swid*(MiddleValue/2-1)/MiddleValue,NowPoint[0][3]);

        StaticGray.get(YOffset,swid*(MiddleValue/2+1)/MiddleValue,NowPoint[1][0]);
        StaticGray.get(YOffset+SubMatHight,swid*(MiddleValue/2+1)/MiddleValue,NowPoint[1][1]);
        StaticGray.get(YOffset,swid/2+RMax-1,NowPoint[1][2]);
        StaticGray.get(YOffset+SubMatHight,swid/2+RMax-1,NowPoint[1][3]);
        for(int i=0;i<8;i++){
            Points[NowFrame%3][i/4][i%4] = NowPoint[i/4][i%4][0];
        }
        Points[NowFrame%3][0][4] = LMax;
        Points[NowFrame%3][1][4] = RMax;
        int TotalPV = 0;
        int varpl[] = new int[5];
        for(int i=0;i<5;i++){
            int varpp = (Points[0][1][i]+Points[1][1][i]+Points[2][1][i])/3;
            varpl[i] = (int)Math.sqrt((Points[0][1][i]-varpp)*(Points[0][1][i]-varpp)+(Points[1][1][i]-varpp)*(Points[1][1][i]-varpp)+(Points[2][1][i]-varpp)*(Points[2][1][i]-varpp));
            if(varpl[i]>PointMin) TotalPV++;
        }
        if(TotalPV>0) {
            LogStr = String.format(Locale.CHINA, "静参点更新 %d=%d %d %d %d %d",TotalPV,varpl[0],varpl[1],varpl[2],varpl[3],varpl[4]);
            FileLogs.i(logtag, LogStr);
            getclose = false;
        }
        else{
            if(!getclose){
                getclose = true;
                LogStr = String.format(Locale.CHINA, "静参点 %d %d %d %d %d %d %d %d LMX=%d RMX=%d", Points[0][0][0], Points[0][0][1], Points[0][0][2], Points[0][0][3],
                        Points[0][1][0], Points[0][1][1], Points[0][1][2], Points[0][1][3], Points[0][0][4], Points[0][1][4]);
                FileLogs.i(logtag, LogStr);
            }
        }
        StaticGray.release();
    }

    //0 其他未预料情况 1 中间有趋近点 两边忽略 2 中间没有趋近点，两边有趋近点 3八个点均偏差大
    private int StaticAnalysis(Mat NowMat){
        byte NowPoint[][][] = new byte[2][4][1];
        Mat StaticGray = new Mat();
        Imgproc.cvtColor(NowMat, StaticGray, COLOR_BGR2GRAY);
        StaticGray.get(YOffset,swid/2-LMax,NowPoint[0][0]);
        StaticGray.get(YOffset+SubMatHight,swid/2-LMax,NowPoint[0][1]);
        StaticGray.get(YOffset,swid*(MiddleValue/2-1)/MiddleValue,NowPoint[0][2]);
        StaticGray.get(YOffset+SubMatHight,swid*(MiddleValue/2-1)/MiddleValue,NowPoint[0][3]);

        StaticGray.get(YOffset,swid*(MiddleValue/2+1)/MiddleValue,NowPoint[1][0]);
        StaticGray.get(YOffset+SubMatHight,swid*(MiddleValue/2+1)/MiddleValue,NowPoint[1][1]);
        StaticGray.get(YOffset,swid/2+RMax-1,NowPoint[1][2]);
        StaticGray.get(YOffset+SubMatHight,swid/2+RMax-1,NowPoint[1][3]);
        StaticGray.release();
        int LPV = 0;
        int RPV = 0;
        int MidPV = 0;
        for(int i=2;i<6;i++){
            if(Math.abs(NowPoint[i / 4][i % 4][0]-Points[0][i / 4][i % 4])<VarMin) {
                MidPV++;
            }
        }
        for(int i=0;i<2;i++){
            if(Math.abs(NowPoint[i / 4][i % 4][0]-Points[0][i / 4][i % 4])<VarMin) {
                LPV++;
            }
        }
        for(int i=6;i<8;i++){
            if(Math.abs(NowPoint[i / 4][i % 4][0]-Points[0][i / 4][i % 4])<VarMin) {
                RPV++;
            }
        }
        if(MidPV>1||(LPV>0&&RPV>0)) return 1;
        if(MidPV==1||LPV>0||RPV>0) return 2;
        if(MidPV==0&&LPV==0&&RPV==0) return 3;
        return 0;
    }

    private void analyrun(){
        int AnalyResult = -1;
        while(runflag) {
            try {
                if(getclose){
                    AnalyResult = StaticAnalysis(mRgba);
                    if(AnalyResult==1){
                        if(!ifclose) {
                            ifclose = true;
                            LogStr = "AnalyResult==1 中间有相同";
                            FileLogs.i(logtag, LogStr);
                        }
                    }
                    else if(AnalyResult==2){
                        if(NowState==5){
                            if(ifclose) {
                                LogStr = "AnalyResult==2 两边有相同 NowState==3/5 动态为开";
                                FileLogs.i(logtag, LogStr);
                                ifclose = false;
                            }
                        }
                        else if(NowState==2||NowState==4){
                            if(!ifclose) {
                                ifclose = true;
                                LogStr = "AnalyResult==2 两边有相同 NowState~=3/5 中间有干扰，但是可以推测为关闭";
                                FileLogs.i(logtag, LogStr);
                            }
                        }
                    }
                    else if(AnalyResult==3){
                        ifclose = false;
                        LogStr = "AnalyResult==3 两边中间均不相同，推测为开";
                        FileLogs.i(logtag, LogStr);
                    }
                    else{
                        LogStr = "出现意外静态对比值";
                        FileLogs.i(logtag, LogStr);
                    }
                    if(NowState==4) ifclose = true;
                }
                else{
                    if(NowState==2||NowState==4) ifclose = true;
                    else if(NowState==3||NowState==5) ifclose = false;
                }
                if(ifclose!=iflight)//光机状态应该和门状态相同，门关闭为true，光机开启为true，门关闭为false（打开状态），光机为false（关闭状态）
                {
                    LogStr = String.format(Locale.CHINA, "当前状态改变 AnalyResult = %d ifclose = %d iflight= %d", AnalyResult, ifclose?1:0, iflight?1:0);
                    FileLogs.i(logtag,LogStr);
                    //查询光机状态
                    CmdResult = CmdExec.execCommand("cat /sys/class/leds/lcd-backlight/brightness",false);
                    if(CmdResult.result>=0) {
                        lightstatus = true;
                        if (CmdResult.successMsg.equals("0")) {
                            iflight = false;
                            LogStr = "查询到光机关闭";
                            FileLogs.i(logtag,LogStr);
                        }
                        else {
                            iflight = true;
                            LogStr = "查询到光机开启";
                            FileLogs.i(logtag,LogStr);
                        }
                        if(ifclose!=iflight){
                            LogStr = String.format(Locale.CHINA, "确认当前状态改变，发送切换命令,从%s到%s", iflight?"开启":"关闭", iflight?"关闭":"开启");
                            FileLogs.i(logtag,LogStr);
                            OnOffScreen();
                            iflight = !iflight;
                        }
                    }
                    else{
                        lightstatus = false;
                        iflight = true;
                    }

                }
                Thread.sleep(100);
            }
            catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        LogStr = "线程A退出";
        FileLogs.i(logtag,LogStr);
    }
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


        Thread threaanaly = new Thread("ThreadAnaly") {
            public void run(){  analyrun(); }
        };
        threaanaly.start();
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
        mRgba.copyTo(mFrame);
        nFrmNum++;
        if(nFrmNum>=(Integer.MAX_VALUE)) nFrmNum = 0;//按每秒30帧算，Integer.MAX_VALUE=2147483647，折合828天半。超过这个运行时间，计数器归零，且不影响循环判断。
        Imgproc.line(mFrame,new Point(swid/2,YOffset),new Point(swid/2,YOffset+SubMatHight),new Scalar(0,0,255),2);
        Imgproc.line(mFrame,new Point(swid*(MiddleValue/2-1)/MiddleValue,YOffset+SubMatHight/2),new Point(swid*(MiddleValue/2+1)/MiddleValue,YOffset+SubMatHight/2),new Scalar(255,255,0),1);

        Imgproc.line(mFrame,new Point(swid/2-LMax,YOffset),new Point(swid/2-LMax,YOffset+SubMatHight),new Scalar(0,255,0),1);

        Imgproc.line(mFrame,new Point(swid/2+RMax-1,YOffset),new Point(swid/2+RMax-1,YOffset+SubMatHight),new Scalar(0,255,0),1);
        //if(Lwidth>0) Imgproc.rectangle(mFrame,new Point(LL,YOffset),new Point(LR,YOffset+SubMatHight),new Scalar(255,255,0),3);
        //if(Rwidth>0) Imgproc.rectangle(mFrame,new Point(RL,YOffset),new Point(RR,YOffset+SubMatHight),new Scalar(255,255,0),3);

        return mFrame;//此处必须返回与控件相同分辨率的mat，否则无法显示。
    }
}
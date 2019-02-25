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
    private int TotalDiff,TotalArea,AreaVar,NowState,DiffCount;
    private int Lwidth,Rwidth,LtoR,LSpeed,RSpeed;
    private int MiddleValue,SideValue,LMax,RMax,VarMin,PointMin,SpeedMin,SpeedMax;
    private int DiffBuffer[][];
    private int Points[][][];
    private boolean runflag,speedflag,getclose,ifclose,lightstatus,iflight;
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
        DiffBuffer = new int[FrameNum][6];              //保持之前的运动状态的缓存
        for(int i=0;i<FrameNum;i++){
            DiffBuffer[i][0] = 2;                          //变化区块的数量
            DiffBuffer[i][1] = 0;                          //变化区域的面积
            DiffBuffer[i][2] = 0;                          //左侧中间未变化的长度
            DiffBuffer[i][3] = 0;                          //右侧中间未变化的长度
            DiffBuffer[i][4] = 0;                          //左侧长度的变化速率
            DiffBuffer[i][5] = 0;                          //右侧长度的变化速率
        }

        Points = new int[3][2][5];                //每扇门四个极限点,一个极限长，两扇门，三组作为基准对比，误差全小于Pointvar则为准备好。
        for(int i=0;i<30;i++){
            Points[i/10][(i/5)%2][i%5] = 0;
        }
        TotalDiff = FrameNum*2;
        TotalArea = 0;
        AreaVar = 0;
        MiddleValue = 20;
        SideValue = 12;
        VarMin = 50;
        PointMin = 30;
        LMax=RMax=swid/2;
        LSpeed = RSpeed = 0;
        DiffCount = 0;
        SpeedMin = 10;
        SpeedMax = swid/4;
        nFrmNum = nFrmSync = nGetPoint = 0;//初始化帧数计数器及同步计数器
        nProcNum = 1;//初始化帧处理计数器
        NowState = 0;
        runflag = true;
        getclose  = false;
        speedflag = false;
        ifclose = false;
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
            NowArea += boundRect[i].width*boundRect[i].height;
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

        int TotalVar = 0;
        for(int i=0;i<FrameNum;i++){
            TotalVar += (DiffBuffer[i][1]-TotalArea/FrameNum)*(DiffBuffer[i][1]-TotalArea/FrameNum);
        }
        AreaVar = (int)Math.sqrt(TotalVar/FrameNum);

        DiffBuffer[NowFrame%FrameNum][2] = Lwidth;                     //更新L无差异距离
        DiffBuffer[NowFrame%FrameNum][3] = Rwidth;                     //更新R无差异距离
        if(NowFrame>=2) {
            LSpeed = (DiffBuffer[(NowFrame - 2) % FrameNum][2] - Lwidth) / 2;
            RSpeed = (DiffBuffer[(NowFrame - 2) % FrameNum][3] - Rwidth) / 2;
        }
        DiffBuffer[NowFrame%FrameNum][4] = LSpeed;                     //更新L变化速率（前两帧平均）
        DiffBuffer[NowFrame%FrameNum][5] = RSpeed;                     //更新R变化速率（前两帧平均）
        if(NowFrame>5){
            boolean tempflag = true;
            for(int i=0;i<6;i++) {
                if(DiffBuffer[(NowFrame-i/2)%FrameNum][4+i%2]<SpeedMin||DiffBuffer[(NowFrame-i/2)%FrameNum][4+i%2]>SpeedMax){
                    tempflag = false;
                    break;
                }
            }
            if(speedflag==false&&tempflag==true){           //speedflag即将发生从false到true的变化，代表检测到门刚进入关门状态，记录当前的(实际上是四帧之前)LRMax值
                LMax = DiffBuffer[(NowFrame-4)%FrameNum][2];
                RMax = DiffBuffer[(NowFrame-4)%FrameNum][3];
                ifclose = true;
            }
            speedflag = tempflag;//采用临时布尔值覆盖全局标识，避免其他线程检测全局标识时，标识与当前状态不一致。
        }
        if(LtoR != Lwidth+Rwidth) {
            LtoR = Lwidth+Rwidth;
            LogStr = String.format(Locale.CHINA, "Lwidth:%d Rwidth:%d  LSpeed:%d RSpeed:%d", Lwidth, Rwidth, LSpeed,RSpeed);
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
                    if(ifclose&&!getclose){
                        LogStr = getCacheDir().getAbsolutePath() + "/Close.bmp";
                        Imgcodecs.imwrite(LogStr, mFrame);
                        FileLogs.i(logtag, LogStr);
                        getclose = true;
                    }
                    m_diff = ProcessFrame(mRgba, mLast);
                    AnalyseDiff(m_diff,nProcNum);
                    switch (NowState){
                        case 0:             //初始化状态
                            if(TotalDiff==0){           //检测到第一次静止
                                LogStr = String.format(Locale.CHINA, "首次静止0-1");
                                FileLogs.i(logtag,LogStr);
                                DiffCount = 0;
                                NowState = 1;
                                LogStr = getCacheDir().getAbsolutePath() + "/Static.bmp";
                                Imgcodecs.imwrite(LogStr, mFrame);
                                FileLogs.i(logtag, LogStr);
                            }
                            mRgba.copyTo(mLast);
                            break;
                        case 1:             //首次静止状态   这里没有考虑到一直静止！！！！！！！！！！！！！！
                            if(speedflag&&Lwidth>0&&Rwidth>0){          //检测到关门运动
                                LogStr = String.format(Locale.CHINA, "关门运动1-2");
                                FileLogs.i(logtag,LogStr);
                                DiffCount = 0;
                                NowState = 2;
                            }
                            else if(LtoR==0){               //未检测到关门运动，但是中间区域已经全部变化，可推测为开门或者干扰
                                LogStr = String.format(Locale.CHINA, "非关门运动，开门或者干扰1-3");
                                FileLogs.i(logtag,LogStr);
                                DiffCount = 0;
                                nGetPoint = 0;
                                NowState = 3;
                            }
                            else{                           //未检测到关门运动，中间区域尚未变化，但非中间区域有变化，有干扰，累计帧数
                                DiffCount++;//这里没有考虑到一直静止！！！！！！！！！！！！！！！！！！！！！
                            }
                            if(DiffCount>FrameNum*3){       //累计超过一定数量则需要重新检测静态
                                LogStr = String.format(Locale.CHINA, "首次静态有干扰，重置状态到初始1-0");
                                FileLogs.i(logtag,LogStr);
                                mRgba.copyTo(mLast);
                                DiffCount = 0;
                                NowState = 0;
                            }
                            break;
                        case 2:             //关门状态中
                            if(LtoR==0){                    //门已经关闭
                                LogStr = String.format(Locale.CHINA, "门已经关闭2-4");
                                FileLogs.i(logtag,LogStr);
                                mRgba.copyTo(mLast);
                                DiffCount = 0;
                                nGetPoint = 0;
                                NowState = 4;
                            }
                            else if(LtoR==swid){            //门又打开了
                                LogStr = String.format(Locale.CHINA, "门在关闭途中打开2-5");
                                FileLogs.i(logtag,LogStr);
                                mRgba.copyTo(mLast);
                                DiffCount = 0;
                                NowState = 5;
                            }
                            else{                           //未检测到完全关门，也未检测到门重新打开，累计帧数
                                DiffCount++;
                            }
                            if(DiffCount>FrameNum*3){       //累计超过一定数量则需要重新检测静态
                                LogStr = String.format(Locale.CHINA, "关门中有干扰，重置状态到初始2-0");
                                FileLogs.i(logtag,LogStr);
                                mRgba.copyTo(mLast);
                                DiffCount = 0;
                                NowState = 0;
                            }
                            break;
                        case 3:             //开门状态中
                            if(ifclose) {
                                if (ComparePoint(mRgba, mLast) == 15) {
                                    LogStr = String.format(Locale.CHINA, "静态点全变，门打开3-5");
                                    FileLogs.i(logtag, LogStr);
                                    mRgba.copyTo(mLast);
                                    DiffCount = 0;
                                    NowState = 5;
                                }
                            }
                            else if(AreaVar==0&&TotalArea>SubMatHight*swid/SideValue){
                                LogStr = String.format(Locale.CHINA, "变域面积稳定且大于SideValue，门打开3-5");
                                FileLogs.i(logtag,LogStr);
                                mRgba.copyTo(mLast);
                                DiffCount = 0;
                                NowState = 5;
                            }
                            else if(LtoR==swid){            //门在打开过程中又关闭了
                                LogStr = String.format(Locale.CHINA, "门在打开途中关闭3-4");
                                FileLogs.i(logtag,LogStr);
                                mRgba.copyTo(mLast);
                                DiffCount = 0;
                                nGetPoint = 0;
                                NowState = 4;
                            }
                            else{                           //未检测到完全关门，也未检测到门重新打开，累计帧数
                                if ((ComparePoint(mRgba, mLast)&9) == 0){
                                    nGetPoint++;
                                }
                                DiffCount++;
                            }
                            if(DiffCount>FrameNum*3){       //累计超过一定数量则需要重新检测静态
                                if(ifclose) {
                                    if (DiffCount - nGetPoint<FrameNum/2) {            //按位与1001 得到0000，两个side点均匹配
                                        LogStr = String.format(Locale.CHINA, "判定开门动作依然检测到Side静参点未改变，判定当前门未开3-4");
                                        FileLogs.i(logtag, LogStr);
                                        mRgba.copyTo(mLast);
                                        DiffCount = 0;
                                        NowState = 4;
                                    }
                                    else {
                                        LogStr = String.format(Locale.CHINA, "开门动作判定后有干扰，重置状态到初始");
                                        FileLogs.i(logtag, LogStr);
                                        mRgba.copyTo(mLast);
                                        DiffCount = 0;
                                        NowState = 0;
                                    }
                                }
                                else {
                                    LogStr = String.format(Locale.CHINA, "基准检测被干扰，重置状态到初始");
                                    FileLogs.i(logtag, LogStr);
                                    mRgba.copyTo(mLast);
                                    DiffCount = 0;
                                    NowState = 0;
                                }
                            }
                            break;
                        case 4:         //门已关闭
                            if(TotalDiff==0){               //门关闭后，整个检测区域静态，确认门关闭完全
                                LogStr = String.format(Locale.CHINA, "门关闭后，整个检测区域静态，确认门关闭完全4-6");
                                FileLogs.i(logtag,LogStr);
                                mRgba.copyTo(mLast);
                                DiffCount = 0;
                                NowState = 6;
                            }
                            else if(LtoR==0){               //检测到中间区域已经全部变化，可推测为开门或者干扰
                                LogStr = String.format(Locale.CHINA, "关门尚未稳定又开门4-3");
                                FileLogs.i(logtag,LogStr);
                                DiffCount = 0;
                                nGetPoint = 0;
                                NowState = 3;
                            }
                            else{                           //未检测到关门运动，中间区域尚未变化，但非中间区域有变化，有干扰，累计帧数
                                if((ComparePoint(mRgba, mLast)&9) == 0){
                                    nGetPoint++;
                                }
                                DiffCount++;
                            }
                            if(DiffCount>FrameNum*3){       //累计超过一定数量则需要重新检测静态
                                if(ifclose) {
                                    if (DiffCount - nGetPoint<FrameNum/2) {            //按位与1001 得到0000，两个side点均匹配
                                        LogStr = String.format(Locale.CHINA, "关门后有干扰，但是可以检测到Side静参点4-6");
                                        FileLogs.i(logtag, LogStr);
                                        mRgba.copyTo(mLast);
                                        DiffCount = 0;
                                        NowState = 6;
                                    }
                                    else {
                                        LogStr = String.format(Locale.CHINA, "关门后有干扰，重置状态到初始");
                                        FileLogs.i(logtag, LogStr);
                                        mRgba.copyTo(mLast);
                                        DiffCount = 0;
                                        NowState = 0;
                                    }
                                }
                                else {
                                    LogStr = String.format(Locale.CHINA, "基准检测被干扰，重置状态到初始");
                                    FileLogs.i(logtag, LogStr);
                                    mRgba.copyTo(mLast);
                                    DiffCount = 0;
                                    NowState = 0;
                                }
                            }
                            break;
                        case 5:         //门已打开
                            if(speedflag&&Lwidth>0&&Rwidth>0){          //检测到关门运动
                                LogStr = String.format(Locale.CHINA, "关门运动5-2");
                                FileLogs.i(logtag,LogStr);
                                DiffCount = 0;
                                NowState = 2;
                            }
                            else{                           //未检测到关门运动，中间区域尚未变化，但非中间区域有变化，有干扰，累计帧数
                                DiffCount++;
                            }
                            if(DiffCount>FrameNum*3){       //累计超过一定数量则需要重新检测静态
                                LogStr = String.format(Locale.CHINA, "首次静态有干扰，重置状态到初始1-0");
                                FileLogs.i(logtag,LogStr);
                                mRgba.copyTo(mLast);
                                DiffCount = 0;
                                NowState = 0;
                            }
                            break;
                    }
                    nProcNum++;//分离处理及同步避免空帧
                    if(nProcNum>=(Integer.MAX_VALUE)) nProcNum = 10;//按每秒30帧算，Integer.MAX_VALUE=2147483647，折合828天半。超过这个运行时间，计数器归零(避免下标越界，归到10)，且不影响循环判断。
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
            if(NowPoint[0][i][0]!=NowPoint[1][i][0]) result += 2^i;
        }
        return result;
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
        if(nFrmNum>=(Integer.MAX_VALUE)) nFrmNum = 0;//按每秒30帧算，Integer.MAX_VALUE=2147483647，折合828天半。超过这个运行时间，计数器归零，且不影响循环判断。
        if(!getclose) {
            Imgproc.line(mFrame, new Point(swid / 2, YOffset), new Point(swid / 2, YOffset + SubMatHight), new Scalar(0, 0, 255), 2);
            Imgproc.line(mFrame, new Point(swid * (MiddleValue - 1) / (MiddleValue * 2), YOffset + SubMatHight / 2), new Point(swid * (MiddleValue + 1) / (MiddleValue * 2), YOffset + SubMatHight / 2), new Scalar(255, 255, 0), 1);
            if (ifclose) {
                Imgproc.line(mFrame, new Point(swid / 2 - LMax, YOffset), new Point(swid / 2 - LMax, YOffset + SubMatHight), new Scalar(0, 255, 0), 1);
                Imgproc.line(mFrame, new Point(swid / 2 + RMax - 1, YOffset), new Point(swid / 2 + RMax - 1, YOffset + SubMatHight), new Scalar(0, 255, 0), 1);
            }
        }
        nFrmNum++;
        return mFrame;//此处必须返回与控件相同分辨率的mat，否则无法显示。
    }
}
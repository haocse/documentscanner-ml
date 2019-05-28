package com.haotran.documentscanner.view;

import android.annotation.SuppressLint;
import android.app.Activity;
import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.Color;
import android.graphics.ImageFormat;
import android.graphics.Matrix;
import android.graphics.Paint;
import android.graphics.Path;
import android.graphics.Rect;
import android.graphics.RectF;
import android.graphics.drawable.shapes.PathShape;
import android.hardware.Camera;
import android.media.AudioManager;
import android.os.CountDownTimer;
import android.os.Environment;
import android.util.Log;
import android.view.MotionEvent;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.View;
import android.widget.FrameLayout;
import android.widget.ImageView;

import com.haotran.documentscanner.constants.ScanConstants;
import com.haotran.documentscanner.enums.ScanHint;
import com.haotran.documentscanner.interfaces.IScanner;
import com.haotran.documentscanner.util.ImageDetectionProperties;
import com.haotran.documentscanner.util.ScanUtils;

import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.ximgproc.StructuredEdgeDetection;

import java.io.File;
import java.io.IOException;
import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;

import static org.opencv.core.CvType.CV_8UC1;

/**
 * This class previews the live images from the camera
 */

public class ScanSurfaceView extends FrameLayout implements SurfaceHolder.Callback, Camera.AutoFocusCallback {
    private static final String TAG = ScanSurfaceView.class.getSimpleName();
    SurfaceView mSurfaceView;
    private final ScanCanvasView scanCanvasView;
    private int vWidth = 0;
    private int vHeight = 0;

    private final Context context;
    private Camera camera;

    private final IScanner iScanner;
    private CountDownTimer autoCaptureTimer;
    private int secondsLeft;
    private boolean isAutoCaptureScheduled;
    private Camera.Size previewSize;
    private boolean isCapturing = false;
    ImageView imageView;
    ImageView imageView2;
    ImageView imageView3;


    StructuredEdgeDetection edgeDetection;


    @SuppressLint("ClickableViewAccessibility")
    public ScanSurfaceView(Context context, IScanner iScanner, ImageView imageView,ImageView imageView2,ImageView imageView3, StructuredEdgeDetection edgeDetection) {
        super(context);
        mSurfaceView = new SurfaceView(context);
        addView(mSurfaceView);
        this.context = context;
        this.scanCanvasView = new ScanCanvasView(context);
        addView(scanCanvasView);
        SurfaceHolder surfaceHolder = mSurfaceView.getHolder();
        surfaceHolder.addCallback(this);
        this.iScanner = iScanner;
        this.edgeDetection = edgeDetection;


        mSurfaceView.setOnTouchListener(new OnTouchListener() {
            @Override
            public boolean onTouch(View view, MotionEvent motionEvent) {
                try {
                    cameraFocus(motionEvent);
                } catch (Exception e) {
                    Log.e(">>>", e.getMessage());
                }

                return true;
            }
        });
        this.imageView = imageView;
        this.imageView2 = imageView2;
        this.imageView3 = imageView3;
    }

    @Override
    public void surfaceCreated(SurfaceHolder holder) {
        try {
            requestLayout();
            openCamera();
            this.camera.setPreviewDisplay(holder);
            setPreviewCallback();
        } catch (IOException e) {
            Log.e(TAG, e.getMessage(), e);
        }
    }

    public void clearAndInvalidateCanvas() {
        scanCanvasView.clear();
        invalidateCanvas();
    }

    public void invalidateCanvas() {
        scanCanvasView.invalidate();
    }

    private void openCamera() {
        if (camera == null) {
            Camera.CameraInfo info = new Camera.CameraInfo();
            int defaultCameraId = 0;
            for (int i = 0; i < Camera.getNumberOfCameras(); i++) {
                Camera.getCameraInfo(i, info);
                if (info.facing == Camera.CameraInfo.CAMERA_FACING_BACK) {
                    defaultCameraId = i;
                    break;
                }
            }
            camera = Camera.open(defaultCameraId);
            if (info.canDisableShutterSound) {
                camera.enableShutterSound(false);
            }
            Camera.Parameters cameraParams = camera.getParameters();

            List<String> flashModes = cameraParams.getSupportedFlashModes();
            if (null != flashModes && flashModes.contains(Camera.Parameters.FLASH_MODE_AUTO)) {
                cameraParams.setFlashMode(Camera.Parameters.FLASH_MODE_AUTO);
            }

            camera.setParameters(cameraParams);
        }
    }



    @Override
    public void surfaceChanged(SurfaceHolder holder, int format, int width, int height) {
        if (vWidth == vHeight) {
            return;
        }
        if (previewSize == null)
            previewSize = ScanUtils.getOptimalPreviewSize(camera, vWidth, vHeight);

        Camera.Parameters parameters = camera.getParameters();

        List<Camera.Size> previewSizes = parameters.getSupportedPreviewSizes();


        camera.setDisplayOrientation(ScanUtils.configureCameraAngle((Activity) context));
        Log.d(">>>", "width: " + previewSize.width);
        Log.d(">>>", "height: " + previewSize.height);



//        List<Camera.Size> allSizes = parameters.getSupportedPreviewSizes();
//        Camera.Size size = allSizes.get(0); // get top size
//        for (int i = 0; i < allSizes.size(); i++) {
//            if (allSizes.get(i).width > size.width)
//                size = allSizes.get(i);
//        }

        Log.d(">>>", "size: " + previewSizes.size());
        int valu = 2;
//        Log.d(">>>", "width1: " + previewSizes.get(valu).width);
//        Log.d(">>>", "height1: " + previewSizes.get(valu).height);


//        parameters.setPreviewSize(size.width, size.height);
//
//
//        parameters.setPreviewSize(previewSize.width, previewSize.height);
//        parameters.setPreviewSize(previewSizes.get(valu).width, previewSizes.get(valu).height);

        if (parameters.getSupportedFocusModes() != null
                && parameters.getSupportedFocusModes().contains(Camera.Parameters.FOCUS_MODE_CONTINUOUS_PICTURE)) {
            parameters.setFocusMode(Camera.Parameters.FOCUS_MODE_CONTINUOUS_PICTURE);
        } else
            if (parameters.getSupportedFocusModes() != null
                && parameters.getSupportedFocusModes().contains(Camera.Parameters.FOCUS_MODE_AUTO)) {
            parameters.setFocusMode(Camera.Parameters.FOCUS_MODE_AUTO);
        }



        Camera.Size sizeS = ScanUtils.determinePictureSize(camera, parameters.getPreviewSize());


        List<Camera.Size> allSizes = parameters.getSupportedPictureSizes();
        int i = 3;
//        parameters.setPictureSize(allSizes.get(i).width, allSizes.get(i).height);
        parameters.setPictureSize(sizeS.width, sizeS.height);
//        int i = 0;
//        parameters.setPictureSize(allSizes.get(i).width, allSizes.get(i).height);
//        Log.d(">>>", "width: " + size.width);
//        Log.d(">>>", "height: " + size.height);
        parameters.setPictureFormat(ImageFormat.JPEG);

        camera.setParameters(parameters);
        requestLayout();
        setPreviewCallback();
    }

    @Override
    public void surfaceDestroyed(SurfaceHolder holder) {
        stopPreviewAndFreeCamera();
    }

    private void stopPreviewAndFreeCamera() {
        if (camera != null) {
            // Call stopPreview() to stop updating the preview surface.
            camera.stopPreview();
            camera.setPreviewCallback(null);
            // Important: Call release() to release the camera for use by other
            // applications. Applications should release the camera immediately
            // during onPause() and re-open() it during onResume()).
            camera.release();
            camera = null;
        }
    }

    public void setPreviewCallback() {
        this.camera.startPreview();
        this.camera.setPreviewCallback(previewCallback); // for receive data.
    }

    public void SaveImage (Mat mat) {
        Mat mIntermediateMat = new Mat();

        Imgproc.cvtColor(mat, mIntermediateMat, Imgproc.COLOR_RGBA2BGR, 3);

        File path = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_PICTURES);
        String filename = "barry.png";
        File file = new File(path, filename);

        Boolean bool = null;
        filename = file.toString();
        bool = Imgcodecs.imwrite(filename, mIntermediateMat);

        if (bool == true)
            Log.d(TAG, "SUCCESS writing image to external storage");
        else
            Log.d(TAG, "Fail writing image to external storage");
    }

    long currentTime = System.currentTimeMillis();
    private final Camera.PreviewCallback previewCallback = new Camera.PreviewCallback() {
        @Override
        public void onPreviewFrame(final byte[] data, final Camera camera) {
            long now = System.currentTimeMillis() - currentTime;
            if (now < 500) {
                return;
            } else {
                currentTime = System.currentTimeMillis();
            }
            if (null != camera ) {
                //Convert try to async ....

//                AsyncTask.execute(new Runnable() {
//                    @Override
//                    public void run() {
//
//                    }
//                });

                try {
                    //TODO your background code
                    Camera.Size pictureSize = camera.getParameters().getPreviewSize();
                    int width = pictureSize.width;
                    int height = pictureSize.height;

                    Log.d(TAG, "onPreviewFrame - received image " + pictureSize.width + "x" + pictureSize.height);
                    Mat yuv = new Mat(new Size(pictureSize.width, pictureSize.height * 1.5), CV_8UC1);
                    yuv.put(0, 0, data);

                    Mat mat = new Mat(new Size(pictureSize.width, pictureSize.height), CvType.CV_8UC4);
                    Imgproc.cvtColor(yuv, mat, Imgproc.COLOR_YUV2BGR_NV21, 4);


//                    Mat mat = yuv;

//                    Imgproc.cvtColor(yuv, mat, Imgproc.COLOR_YUV2GRAY_NV21, 4);
//                    Imgproc.cvtColor(mat, mat, Imgproc.COLOR_BGR2GRAY);
//                    Imgproc.GaussianBlur(mat, mat, new Size(5,5), 0);
//                    Imgproc.cvtColor(mat, mat, Imgproc.COLOR_RGB2GRAY);
//                    Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB, 4);


//                    Imgcodecs.imwrite("Gaussian45.jpg", destination);

//                    SaveImage(mat);

                    yuv.release();




//                    YuvImage yuv = new YuvImage(data,camera.getParameters().getPreviewFormat(), width, height, null);
//                    ByteArrayOutputStream out = new ByteArrayOutputStream();
//                    yuv.compressToJpeg(new Rect(0, 0, width, height), 100, out);
//                    byte[] bytes = out.toByteArray();
//                    Bitmap bitmap = BitmapFactory.decodeByteArray(bytes, 0, bytes.length);
//                    Mat mat = new Mat();
//                    Utils.bitmapToMat(bitmap, mat);
//                    bitmap.recycle();



                    final Size originalPreviewSize = mat.size();
                    final int originalPreviewArea = mat.rows() * mat.cols();
                    double scale = 0.2;

                    double newWidth = mat.size().width * scale;
                    double newHeight = mat.size().height * scale;
                    Size sz = new Size(newWidth,newHeight);
//                    double originalWidth = srcImage.size().width;
//                    double originalHeight = srcImage.size().height;
//                    double newWidth = srcImage.size().width * scale;
//                    double newHeight = srcImage.size().height * scale;


                    // split this function to multiple lines.

//                    List<MatOfPoint> largestContour = ScanUtils.findLargestContour(mat);
                    Mat dilate = ScanUtils.findLargestMatWithEdgeDetection(mat, edgeDetection); //small size
//                    Mat dilate;
//                    dilate = ScanUtils.findLargestMat(mat);

                    final Bitmap bm = Bitmap.createBitmap(dilate.cols(), dilate.rows(),Bitmap.Config.ARGB_8888);
                    Utils.matToBitmap(dilate, bm);

//                    // make lines here
                    Mat lines = new Mat();
                    long houghTime = System.currentTimeMillis();
//                    Imgproc.HoughLinesP(dilate, lines, 1, Math.PI / 180, 65, 10, 20); // threshold 65
                     //replace HoughLinesP to HoughLines.
//                    Imgproc.HoughLines(dilate, lines, 1, Math.PI / 180, 65/*, 100, 20*/); // threshold 65
//                    Imgproc.HoughLines(dilate, lines, 1, Math.PI / 180, 50/*, 100, 20*/); // threshold 65
                    Imgproc.HoughLinesP(dilate, lines, 1, Math.PI / 180, 65, 25, 4); // threshold 65
                    // 100 20
                    // 30 4
                    Log.d(">>>", "Hough Time: " + (System.currentTimeMillis() - houghTime) + "");
//                    Log.d(">>>", "Hough Time: " + (System.currentTimeMillis() - houghTime) + "");

                    Mat houghLines = new Mat(new Size(dilate.size().width, dilate.size().height), CvType.CV_8UC1);
                    houghLines.create(dilate.rows(), dilate.cols(), CvType.CV_8UC1);
                    //Drawing lines on the image

//                    for (int x = 0; x < lines.rows(); x++) { //count the rows..
//                        double rho = lines.get(x, 0)[0],
//                                theta = lines.get(x, 0)[1];
//                        double a = Math.cos(theta), b = Math.sin(theta);
//                        double x0 = a*rho, y0 = b*rho;
//                        Point pt1 = new Point(Math.round(x0 + 1000*(-b)), Math.round(y0 + 1000*(a)));
//                        Point pt2 = new Point(Math.round(x0 - 1000*(-b)), Math.round(y0 - 1000*(a)));
//                        Imgproc.line(houghLines, pt1, pt2, new Scalar(255, 0, 255), 3, Imgproc.LINE_AA, 0);
//                    }

//                    for (int i = 0; i < lines.rows(); i++) {
//                        double[] points = lines.get(i, 0);
//                        double x1, y1, x2, y2;
//
//                        x1 = points[0];
//                        y1 = points[1];
//                        x2 = points[2];
//                        y2 = points[3];
//
//                        Log.d(">>>", "Coordinates: x1=>"+x1+" y1=>"+y1+" x2=>"+x2+" y2=>"+y2);
//
//                        Point pt1 = new Point(x1, y1);
//                        Point pt2 = new Point(x2, y2);
//
//                        //Drawing lines on an image
//                        double dx = x1 - x2;
//                        double dy = y1 - y2;
//                        double dist = Math.sqrt (dx*dx + dy*dy);
////                        if(dist>300.d)
//                            Imgproc.line(houghLines, pt1, pt2, new Scalar(255, 0, 255), 1, Imgproc.LINE_AA);
//                    }
//
//                    //Converting Mat back to Bitmap

                    int group_similar_thr=30;

//                    for (int x = 0; x < lines.rows(); x++) {
//                        // convert line to array list of `rho` + `theta`...
////                        double rho = lines.get(x, 0)[0],
////                                theta = lines.get(x, 0)[1];
////                        _lines.add(new Line(rho, theta));
//                        Log.d(">>>line", lines.get(x, 0)[0] + "");
//                        Log.d(">>>line", lines.get(x, 0)[1] + "");
//                        Log.d(">>>line", lines.get(x, 0)[2] + "");
//                        Log.d(">>>line", lines.get(x, 0)[3] + "");
//
//                        // convert to rho and theta.. .
//
//                    }
//                    Log.d(">>>lines", lines.toString());
                    ArrayList<Line> _lines = _cvhoughlines2list(lines);

                    if (group_similar_thr != 0) {
                        _lines = _group_similar(_lines, group_similar_thr);
                    }

                    Log.d(">>>", dilate.width() + " rows");
                    Log.d(">>>", dilate.height() + " height");

//                    ArrayList<LinePair> lines2 = find_intersections(_lines, dilate);

                    ArrayList<Intersection> dots = find_intersections2(_lines, dilate);

                    /*quad = */find_quadrilaterals(dots);


                    // draw here.
                    // for testing
//                    for (int x = 0; x < lines2.size(); x++) { //count the rows..
//                        double rho = lines2.get(x).getLine1().getRho(),
//                                theta = lines2.get(x).getLine1().getTheta();
//                        double a = Math.cos(theta) /*x = x0 + r*cos(theta)*/, b = Math.sin(theta);/*y = y0 + r*sin(theta)*/
//                        double x0 = a*rho, y0 = b*rho;
//
//                        Point pt1 = new Point(Math.round(x0 + 1000*(-b)), Math.round(y0 + 1000*(a)));
//                        Point pt2 = new Point(Math.round(x0 - 1000*(-b)), Math.round(y0 - 1000*(a)));
//                        Imgproc.line(houghLines, pt1, pt2, new Scalar(255, 0, 255), 3, Imgproc.LINE_AA, 0);
//                    }

                    // draw dots
                    for (int x = 0; x < dots.size(); x++) { //count the rows..

                        Imgproc.circle(houghLines, dots.get(x).getCoords(), 10, new Scalar(255, 0, 255), 3);
                    }

                    File path = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_PICTURES);
                    String filename = "lines.png";
                    File file = new File(path, filename);

                    Boolean bool = null;
                    filename = file.toString();
                    Imgcodecs.imwrite(filename, houghLines);

                    final Bitmap currentBitmap = Bitmap.createBitmap(dilate.cols(), dilate.rows(),Bitmap.Config.ARGB_8888);
                    Utils.matToBitmap(houghLines, currentBitmap);

                    imageView.setImageBitmap(currentBitmap);
                    imageView2.setImageBitmap(bm);
                    houghLines.release();
//                    currentBitmap.recycle();

//                    imageView.setImageBitmap(bm);



                    filename = "processed.png";
                    file = new File(path, filename);
                    filename = file.toString();
                    Imgcodecs.imwrite(filename, dilate);

                    Mat dilate2;
                    dilate2 = ScanUtils.findLargestMat(mat);

//                    Log.d(">>>", "width: " + newWidth);
//                    Log.d(">>>", "height: " + newHeight);
                    Imgproc.resize( dilate2, dilate2, sz);
                    final Bitmap bm2 = Bitmap.createBitmap(dilate2.cols(), dilate2.rows(),Bitmap.Config.ARGB_8888);
                    Utils.matToBitmap(dilate2, bm2);
                    imageView3.setImageBitmap(bm2);
                    dilate2.release();

                    Size sz2 = new Size(originalPreviewSize.width,originalPreviewSize.height);
                    Imgproc.resize(dilate, dilate, sz2);




                    // find the imageview and draw it!
//                    ImageView iv = (ImageView) findViewById(R.id.imageView1);

//                    ((Activity)context).runOnUiThread(new Runnable() {
//                        @Override
//                        public void run() {
//                            // update UI
////                                runInBackground();
//
//
//                        }
//                    });





                    Quadrilateral largestQuad = null; //ScanUtils.detectLargestQuadrilateral(mat);
                    List<MatOfPoint> largestContour = ScanUtils.findLargestContourFromMat(dilate);
                    if (null != largestContour) {
                        Quadrilateral mLargestRect = ScanUtils.findQuadrilateral(largestContour);
                        if (mLargestRect != null)
                            largestQuad = mLargestRect;
                    }

//                    Quadrilateral largestQuad = ScanUtils.detectLargestQuadrilateral(img);
                    clearAndInvalidateCanvas();
//                    img.release();

                    mat.release();
                    dilate.release();

                    final Quadrilateral finalLargestQuad = largestQuad;
                    if (null != finalLargestQuad) {
                        drawLargestRect(finalLargestQuad.contour, finalLargestQuad.points, originalPreviewSize, originalPreviewArea);
                    } else {
                        showFindingReceiptHint();
                    }

                } catch (Exception e) {
                    Log.e(">>>", e.getMessage());
                    showFindingReceiptHint();
                }
            }
        }
    };

    private void find_quadrilaterals(ArrayList<Intersection> intersections) {
        HashMap<Integer, ArrayList<Integer>> graph = build_graph(intersections);
//        int[] loops = new int[10];
//        int[] seen = new int[10];

        ArrayList<Integer> loops = new ArrayList<>();
        ArrayList<Integer> seen = new ArrayList<>();

//        ArrayList<Object> loops = new ArrayList<>();
        for(int node : graph.keySet()) {
            _bounded_dfs(graph, node, loops, seen);
        }

//        Log.d(">>>loop", loops.get(0) + "");
//        for (int i = 0; i < loops.size(); i++) {
//            Log.d(">>>loop", i + ":" + loops.get(i) + "");
//        }

        Log.d(">>>loo", loops.size() + "");


        /**
         * for node in graph:
         * _bounded_dfs(graph, node, loops)
         */

        // convert loops to coords

    }

    private void _bounded_dfs(HashMap<Integer, ArrayList<Integer>> neighbours, int current, ArrayList<Integer> loops,  ArrayList<Integer> seen/*list of ids*/) { //
//        loops.add(1000);
//        loops.add(2);

//        if current in seen:
//        # print ("return...")
//        return

        if (seen.contains(current)) {
            return;
        }
        seen.add(current);
        if (seen.size() == 4) {
            _add_if_loop(current, neighbours, seen, loops);
        } else {
//            [_bounded_dfs(neighbours, neighbour, loops, seen=seen) for neighbour in neighbours[current]]
            for (int neighbour : neighbours.keySet()) {
                _bounded_dfs(neighbours, neighbour, loops, seen);
            }

        }

        // remove the last one...
        seen.remove(seen.size()-1);

    }

    private void _add_if_loop(int current, HashMap<Integer, ArrayList<Integer>> neighbours, ArrayList<Integer> seen, ArrayList<Integer> loops) {
        // if seen[0] in neighbours[current]:
        if (neighbours.get(current).contains(seen.get(0))) {
            loops.addAll(seen);
        }
    }

    private void _bounded_dfs(HashMap<Integer, ArrayList<Integer>> graph, int node, int[] loops, int[] seen) {

    }

    private HashMap<Integer, ArrayList<Integer>> build_graph(ArrayList<Intersection> intersections) {
        /*
        graph = {k["id"]: [] for k in intersections}
        for i1, i2 in itertools.permutations(intersections, 2):
        if _common_line_exists(i1["lines"], i2["lines"]):
        graph[i1["id"]].append(i2["id"])
        return graph
        */

        Log.d(">>>intersections", intersections.toString());

        // 1. loop into intersections => a list of id
        HashMap<Integer, ArrayList<Integer>> graph = new HashMap<Integer, ArrayList<Integer>>();

        for (int i = 0; i < intersections.size(); i++) {
            graph.put(intersections.get(i).getId(), new ArrayList<Integer>());
//            graph[intersections.get(i).getId()] = new ArrayList<>();
        }

        int size = intersections.size();
        for (int i = 0; i < size; i++) {
            for (int j = 0; j < size; j++) {
                if (i != j) {
                    Intersection i1 = intersections.get(i);
                    Intersection i2 = intersections.get(j);
                    if (_common_line_exists(i1.getLinePair(), i2.getLinePair())) {
                        Log.d(">>>gra", "exists");
                        // graph id 1 => ...
                        int key = intersections.get(i).getId();
                        ArrayList<Integer> value = graph.get(key);

//                        if (value == null) value = new ArrayList<>();
                        value.add(i2.getId()); // add new value (id) //  graph[i1["id"]].append(i2["id"])
                        graph.put(key, value);
                    }
                }
            }
        }

        Log.d(">>>graph", graph.toString());
//        for (int i = 0; i < intersections.size(); i++) {
//            Line line1 = intersections.get(i).getLinePair().line1;
//            Line line2 = intersections.get(i).getLinePair().line2;
//        }

        // 2. use retainAll to find common line.

        return graph;
    }

    private boolean _common_line_exists(LinePair i1Lines, LinePair i2Lines) {
        // i1lines: line1, line2
        Line i1line1 = i1Lines.line1; // alpha, theta.
        Line i1line2 = i1Lines.line2;

        Line i2line1 = i2Lines.line1;
        Line i2line2 = i2Lines.line2;

//        i1line1.theta//theta
        if ((i1line1.compare(i2line1)) || (i1line1.compare(i2line2)) || (i1line2.compare(i2line1)) || (i1line2.compare(i2line2))) {
            Log.d(">>>>", "true");
        } else {
            Log.d(">>>>", "false");
        }

        return ((i1line1.compare(i2line1)) || (i1line1.compare(i2line2)) || (i1line2.compare(i2line1)) || (i1line2.compare(i2line2)));
//        return false;
    }

    public Point intersection(int x1, int y1, int x2, int y2, int x3, int y3, int x4, int y4) {
        int d = (x1-x2)*(y3-y4) - (y1-y2)*(x3-x4);
        if (d == 0) return new Point(-1, -1);

        int xi = ((x3-x4)*(x1*y2-y1*x2)-(x1-x2)*(x3*y4-y3*x4))/d;
        int yi = ((y3-y4)*(x1*y2-y1*x2)-(y1-y2)*(x3*y4-y3*x4))/d;

        return new Point(xi,yi);
    }

    private ArrayList<Intersection> find_intersections2(ArrayList<Line> lines, Mat dilate) {

//        intersections = [];
//        permutations -> write a customized class for this purpose

        // create a function to make another list that has only 2 items.

        ArrayList<Intersection> intersections = makeIntersections(lines, 45, dilate);


//        combinations2()


        return intersections;
    }

    private  ArrayList<LinePair> find_intersections(ArrayList<Line> lines, Mat dilate) {

//        intersections = [];
//        permutations -> write a customized class for this purpose

        // create a function to make another list that has only 2 items.

        ArrayList<LinePair> linePairs = makeLinePairs(lines, 45, dilate);


//        combinations2()


        Log.d(">>>", "linePairs: " + linePairs.size());
        for (int i = 0; i < lines.size(); i++) {
            // for lines... what to do.

            Line line = lines.get(i);
        }
        int vertex_id = 0;
        return linePairs;
    }

    static void combinations2(String[] arr, int len, int startPosition, String[] result){
        if (len == 0){
            Log.d(">>>","combinations" + Arrays.toString(result));
//            Log.d(">>>", result.)
            return;
        } else {

        }
        for (int i = startPosition; i <= arr.length-len; i++){
            result[result.length - len] = arr[i];
            combinations2(arr, len-1, i+1, result);
        }
    }

    private ArrayList<LinePair> makeLinePairs(ArrayList<Line> lines, int angle, Mat dilate) {
        int height = dilate.height();
        int width = dilate.width();
        // Improve this later using  combinations2...

        ArrayList<Intersection> intersections = new ArrayList<>();
        int vertex_id = 0;
        ArrayList<LinePair> linePairs = new ArrayList<>();
        for (int j = 0; j < lines.size(); j++) {
            for (int k = 0; k < lines.size(); k++) {
                Line line1 = lines.get(j);
                Line line2 = lines.get(k);
                if (j != k && !_angles_are_similar(line1, line2, angle)) {
                    linePairs.add(new LinePair(lines.get(j), lines.get(k)));
                    // and _find_intersection_coords here.
                    Point coords = _find_intersection_coords(line1, line2);
                    if (_coords_are_valid(coords, width, height) && !_already_present(coords, intersections)) {
                        intersections.add(new Intersection(vertex_id, new LinePair(line1, line2), coords));
                        vertex_id++;
                    }
                }
            }
        }
        return linePairs;
    }

    private ArrayList<Intersection> makeIntersections(ArrayList<Line> lines, int angle, Mat dilate) {
        int height = dilate.height();
        int width = dilate.width();
        // Improve this later using  combinations2...

        ArrayList<Intersection> intersections = new ArrayList<>();
        int vertex_id = 0;
//        ArrayList<LinePair> linePairs = new ArrayList<>();
        for (int j = 0; j < lines.size(); j++) {
            for (int k = 0; k < lines.size(); k++) {
                Line line1 = lines.get(j);
                Line line2 = lines.get(k);
                if (j != k && !_angles_are_similar(line1, line2, angle)) {
//                    linePairs.add(new LinePair(lines.get(j), lines.get(k)));
                    // and _find_intersection_coords here.
                    Point coords = _find_intersection_coords(line1, line2);
                    if (_coords_are_valid(coords, width, height) && !_already_present(coords, intersections)) {
                        intersections.add(new Intersection(vertex_id, new LinePair(line1, line2), coords));
                        vertex_id++;
                    }
                }
            }
        }
        return intersections;
    }

    private boolean _already_present(Point coords, ArrayList<Intersection> intersections) {
        for (int i = 0; i < intersections.size(); i++) {
            Intersection intersection = intersections.get(i);
            Point co = intersection.getCoords();
            if (co.x == coords.x && co.y == coords.y) {
                return true;
            }
        }
        return false;
    }

    private boolean _coords_are_valid(Point coords, int width, int height) {
        return 0 < coords.x && coords.x < width && 0 < coords.y && coords.y < height;

    }

    private Point _find_intersection_coords(Line line1, Line line2) {
        Double rho1 = line1.getRho();
        Double theta1 = line1.getTheta();
        Double rho2 = line2.getRho();
        Double theta2 = line2.getTheta();

//        double rho = lines2.get(x).getLine1().getRho(),
//                theta = lines2.get(x).getLine1().getTheta();
//        double a = Math.cos(theta) /*x = x0 + r*cos(theta)*/, b = Math.sin(theta);/*y = y0 + r*sin(theta)*/
//        double x0 = a*rho, y0 = b*rho;
//
//        Point pt1 = new Point(Math.round(x0 + 1000*(-b)), Math.round(y0 + 1000*(a)));
//        Point pt2 = new Point(Math.round(x0 - 1000*(-b)), Math.round(y0 - 1000*(a)));

        double a = Math.cos(theta1) /*x = x0 + r*cos(theta)*/, b = Math.sin(theta1);/*y = y0 + r*sin(theta)*/
        double x0 = a*rho1, y0 = b*rho1;

        int x1 = (int) Math.round(x0 + 1000 * (-b));
        int y1 = (int) Math.round(y0 + 1000 * (a));

        int x2 = (int) Math.round(x0 - 1000*(-b));
        int y2 = (int) Math.round(y0 - 1000*(a));

        a = Math.cos(theta2) /*x = x0 + r*cos(theta)*/;
        b = Math.sin(theta2);/*y = y0 + r*sin(theta)*/
        x0 = a*rho2;
        y0 = b*rho2;

        int x3 = (int) Math.round(x0 + 1000 * (-b));
        int y3 = (int) Math.round(y0 + 1000 * (a));
        int x4 = (int) Math.round(x0 - 1000*(-b));
        int y4 = (int) Math.round(y0 - 1000*(a));


        Point point = intersection(x1, y1, x2, y2, x3, y3, x4, y4);
        return point;
//        Point pt1 = new Point(Math.round(x0 + 1000*(-b)), Math.round(y0 + 1000*(a)));
//        Point pt2 = new Point(Math.round(x0 - 1000*(-b)), Math.round(y0 - 1000*(a)));

//        double a = Math.cos(theta1), b = Math.sin(theta2);
//        double x0 = a*rho, y0 = b*rho;




    }

    private boolean _angles_are_similar(Line line1, Line line2, int angle) {
        Log.d(">>>", "angle: " + lines_angle(line1, line2));
        return lines_angle(line1, line2) < angle;
    }

    private double lines_angle(Line line1, Line line2) {
        return Math.abs(line1.getTheta() - line2.getTheta()) * 180 / Math.PI;
//        return false;
    }

    private ArrayList<Line> _group_similar(ArrayList<Line> lines, int thr) {
//        Comparator<Line> compareById = (Line o1, Line o2) -> o1.getRho().compareTo( o2.getRho());
        Comparator<Line> compareById = new Comparator<Line>() {
            @Override
            public int compare(Line o1, Line o2) {
                return o1.getRho().compareTo(o2.getRho());
            }
        };
        Collections.sort(lines, compareById);

        ArrayList<Line> lines_unique = new ArrayList<>();

        for (int i = 0; i < lines.size(); i++) {
            Line to_add = lines.get(i);
            if (!_is_duplicated(to_add, lines_unique, thr)) {
                lines_unique.add(to_add);
            }
        }


//        lines = sorted(lines, key=lambda line: line[0])
//        lines_unique = []
//        for to_add in lines:
//        if not _is_duplicated(to_add, lines_unique, thr):
//        lines_unique.append(to_add)
        return lines_unique;
    }

    private boolean _is_duplicated(Line line, ArrayList<Line> lines_unique, int thr) {
//        return any(abs(math.fabs(line[0]) - math.fabs(_line[0])) < thr for _line in lines) <- python...
        for (int i = 0; i < lines_unique.size(); i++) {
            Line _line = lines_unique.get(i);
            Log.d(">>>", "abs: " + Math.abs(line.rho - _line.rho));
            if (Math.abs(line.rho - _line.rho) < thr) return true;
        }
        return false;
    }

    private ArrayList<Line> _cvhoughlines2list(Mat lines) {
        //return [(line[0][0], line[0][1]) for line in lines] <- python
        ArrayList<Line> _lines = new ArrayList<>();
//        Log.d(">>>line", lines.get(x, 0)[0] + "");
//        Log.d(">>>line", lines.get(x, 0)[1] + "");
//        Log.d(">>>line", lines.get(x, 0)[2] + "");
//        Log.d(">>>line", lines.get(x, 0)[3] + "");
        for (int x = 0; x < lines.rows(); x++) {
            // convert line to array list of `rho` + `theta`...

            double x1 = lines.get(x, 0)[0];
            double y1 = lines.get(x, 0)[1];
            double x2 = lines.get(x, 0)[2];
            double y2 = lines.get(x, 0)[3];

            double k = x2 - x1;
            if (x2 - x1 == 0) {
                k = 99999;
            }

            double theta = -Math.atan2((x2 - x1), (y2 - y1));
            double rho = x1 * Math.cos(theta) + y1 * Math.sin(theta);

//            double rho = lines.get(x, 0)[0],
//                    theta = lines.get(x, 0)[1];
            _lines.add(new Line(rho, theta));
        }
        return _lines;
    }


    private void drawLargestRect(MatOfPoint2f approx, Point[] points, Size stdSize, int previewArea) {
        Path path = new Path();
        // ATTENTION: axis are swapped
        float previewWidth = (float) stdSize.height;
        float previewHeight = (float) stdSize.width;

//        Log.i(TAG, "previewWidth: " + String.valueOf(previewWidth));
//        Log.i(TAG, "previewHeight: " + String.valueOf(previewHeight));

        //Points are drawn in anticlockwise direction
        path.moveTo(previewWidth - (float) points[0].y, (float) points[0].x);
        path.lineTo(previewWidth - (float) points[1].y, (float) points[1].x);
        path.lineTo(previewWidth - (float) points[2].y, (float) points[2].x);
        path.lineTo(previewWidth - (float) points[3].y, (float) points[3].x);
        path.close();

        double area = Math.abs(Imgproc.contourArea(approx));
        Log.i(TAG, "Contour Area: " + String.valueOf(area));

        PathShape newBox = new PathShape(path, previewWidth, previewHeight);
        Paint paint = new Paint();
        Paint border = new Paint();

        //Height calculated on Y axis
        double resultHeight = points[1].x - points[0].x;
        double bottomHeight = points[2].x - points[3].x;
        if (bottomHeight > resultHeight)
            resultHeight = bottomHeight;

        //Width calculated on X axis
        double resultWidth = points[3].y - points[0].y;
        double bottomWidth = points[2].y - points[1].y;
        if (bottomWidth > resultWidth)
            resultWidth = bottomWidth;

//        Log.i(TAG, "resultWidth: " + String.valueOf(resultWidth));
//        Log.i(TAG, "resultHeight: " + String.valueOf(resultHeight));

        ImageDetectionProperties imgDetectionPropsObj
                = new ImageDetectionProperties(previewWidth, previewHeight, resultWidth, resultHeight,
                previewArea, area, points[0], points[1], points[2], points[3]);

        final ScanHint scanHint;

        if (imgDetectionPropsObj.isDetectedAreaBeyondLimits()) {
            scanHint = ScanHint.FIND_RECT;
            cancelAutoCapture();
        }
//        else if (imgDetectionPropsObj.isDetectedAreaBelowLimits()) {
//            Log.d(">>>", "imgDetectionPropsObj.isDetectedAreaBelowLimits()");
//            cancelAutoCapture();
//            if (imgDetectionPropsObj.isEdgeTouching()) {
//                scanHint = ScanHint.MOVE_AWAY;
//            }
//            else
//            {
//                scanHint = ScanHint.MOVE_CLOSER;
//            }
//        }
        else if (imgDetectionPropsObj.isDetectedHeightAboveLimit()) {
            Log.d(">>>", "imgDetectionPropsObj.isDetectedHeightAboveLimit()");
            cancelAutoCapture();
            scanHint = ScanHint.MOVE_AWAY;
//            scanHint = ScanHint.NO_MESSAGE_WITH_BORDER;
        }
        else if (imgDetectionPropsObj.isDetectedWidthAboveLimit()) {
            Log.d(">>>", "imgDetectionPropsObj.isDetectedWidthAboveLimit()");
            cancelAutoCapture();
            scanHint = ScanHint.MOVE_AWAY;
//            scanHint = ScanHint.NO_MESSAGE_WITH_BORDER;
        }
        else if (imgDetectionPropsObj.isDetectedAreaAboveLimit()) {
            Log.d(">>>", "imgDetectionPropsObj.isDetectedAreaAboveLimit()");
            cancelAutoCapture();
            scanHint = ScanHint.MOVE_AWAY;
//            scanHint = ScanHint.NO_MESSAGE_WITH_BORDER;
        }

        else {
//            if (imgDetectionPropsObj.isEdgeTouching()) {
//                Log.d(">>>", "imgDetectionPropsObj.isEdgeTouching()");
//                cancelAutoCapture();
//                scanHint = ScanHint.MOVE_AWAY;
//            }
//            else if (imgDetectionPropsObj.isAngleNotCorrect(approx)) {
//                Log.d(">>>", "imgDetectionPropsObj.isAngleNotCorrect(approx)");
//                cancelAutoCapture();
//                scanHint = ScanHint.ADJUST_ANGLE;
//            }
//            else
            {
                Log.i(TAG, "GREEN" + "(resultWidth/resultHeight) > 4: " + (resultWidth / resultHeight) +
                        " points[0].x == 0 && points[3].x == 0: " + points[0].x + ": " + points[3].x +
                        " points[2].x == previewHeight && points[1].x == previewHeight: " + points[2].x + ": " + points[1].x +
                        "previewHeight: " + previewHeight);
                scanHint = ScanHint.CAPTURING_IMAGE;
                clearAndInvalidateCanvas();

                if (!isAutoCaptureScheduled) {
                    //Temporarily disable for this one.
//                    scheduleAutoCapture(scanHint);
                }
            }
        }
        Log.i(TAG, "Preview Area 95%: " + 0.95 * previewArea +
                " Preview Area 20%: " + 0.20 * previewArea +
                " Area: " + String.valueOf(area) +
                " Label: " + scanHint.toString());

        border.setStrokeWidth(12);

        iScanner.displayHint(scanHint);
        setPaintAndBorder(scanHint, paint, border);
        scanCanvasView.clear();
        scanCanvasView.addShape(newBox, paint, border);
        invalidateCanvas();
    }

    private void scheduleAutoCapture(final ScanHint scanHint) {
        isAutoCaptureScheduled = true;
        secondsLeft = 0;
        autoCaptureTimer = new CountDownTimer(4000, 100) {
            public void onTick(long millisUntilFinished) {
                if (Math.round((float) millisUntilFinished / 1000.0f) != secondsLeft) {
                    secondsLeft = Math.round((float) millisUntilFinished / 1000.0f);
                }
                Log.v(TAG, "" + millisUntilFinished / 1000);
                switch (secondsLeft) {
                    case 1:
                        autoCapture(scanHint);
                        break;
                    default:
                        break;
                }
            }

            public void onFinish() {
                isAutoCaptureScheduled = false;
            }
        };
        autoCaptureTimer.start();
    }

    @SuppressLint("ClickableViewAccessibility")
    private void autoCapture(ScanHint scanHint) {
        if (isCapturing) return;
        if (ScanHint.CAPTURING_IMAGE.equals(scanHint)) {
            try {
                isCapturing = true;
                iScanner.displayHint(ScanHint.CAPTURING_IMAGE);

                camera.takePicture(mShutterCallBack, null, pictureCallback);
                camera.setPreviewCallback(null);

//                iScanner.displayHint(ScanHint.NO_MESSAGE);
//                clearAndInvalidateCanvas();
                mSurfaceView.setOnTouchListener(new OnTouchListener() {
                    @Override
                    public boolean onTouch(View view, MotionEvent motionEvent) {
                        //do nothing...
                        return true;
                    }
                });
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }

    private void cancelAutoCapture() {
        Log.d(">>>", "Cancelling auto capture");
        if (isAutoCaptureScheduled) {
            isAutoCaptureScheduled = false;
            if (null != autoCaptureTimer) {
                autoCaptureTimer.cancel();
            }
        }
    }

    private void showFindingReceiptHint() {
        iScanner.displayHint(ScanHint.FIND_RECT);
        clearAndInvalidateCanvas();
    }

    private void setPaintAndBorder(ScanHint scanHint, Paint paint, Paint border) {
        int paintColor = 0;
        int borderColor = 0;

        switch (scanHint) {
            case MOVE_CLOSER:
            case MOVE_AWAY:
            case ADJUST_ANGLE:
                paintColor = Color.argb(30, 255, 38, 0);
                borderColor = Color.rgb(255, 38, 0);
                break;
            case FIND_RECT:
                paintColor = Color.argb(0, 0, 0, 0);
                borderColor = Color.argb(0, 0, 0, 0);
                break;
            case CAPTURING_IMAGE:
                paintColor = Color.argb(30, 38, 216, 76);
                borderColor = Color.rgb(38, 216, 76);
                break;
        }

        paint.setColor(paintColor);
        border.setColor(borderColor);
    }

    private final Camera.PictureCallback pictureCallback = new Camera.PictureCallback() {
        @Override
        public void onPictureTaken(byte[] data, Camera camera) {
            camera.stopPreview();
            iScanner.displayHint(ScanHint.NO_MESSAGE);
            clearAndInvalidateCanvas();

            Bitmap bitmap = ScanUtils.decodeBitmapFromByteArray(data,
                    ScanConstants.HIGHER_SAMPLING_THRESHOLD, ScanConstants.HIGHER_SAMPLING_THRESHOLD);

            Matrix matrix = new Matrix();
            matrix.postRotate(90);
            bitmap = Bitmap.createBitmap(bitmap, 0, 0, bitmap.getWidth(), bitmap.getHeight(), matrix, true);

            iScanner.onPictureClicked(bitmap);
            postDelayed(new Runnable() {
                @Override
                public void run() {
                    isCapturing = false;
                }
            }, 3000);

        }
    };

    private final Camera.ShutterCallback mShutterCallBack = new Camera.ShutterCallback() {
        @Override
        public void onShutter() {
            if (context != null) {
                AudioManager mAudioManager = (AudioManager) context.getSystemService(Context.AUDIO_SERVICE);
                if (null != mAudioManager)
                    mAudioManager.playSoundEffect(AudioManager.FLAG_PLAY_SOUND);
            }
        }
    };

    @Override
    protected void onMeasure(int widthMeasureSpec, int heightMeasureSpec) {
        // We purposely disregard child measurements because act as a
        // wrapper to a SurfaceView that centers the camera preview instead
        // of stretching it.
        Log.d(">>>", "onMeasure...");
        vWidth = resolveSize(getSuggestedMinimumWidth(), widthMeasureSpec);
        vHeight = resolveSize(getSuggestedMinimumHeight(), heightMeasureSpec);
        setMeasuredDimension(vWidth, vHeight);
        previewSize = ScanUtils.getOptimalPreviewSize(camera, vWidth, vHeight);


//        if (previewSize != null) {
//            Log.d(">>>", "width: " + previewSize.width);
//            Log.d(">>>", "height: " + previewSize.height);
//            // set preview size here.
//            Camera.Parameters parameters = camera.getParameters();
//
//            parameters.setPreviewSize(previewSize.width, previewSize.height);
//            camera.setParameters(parameters);
//        }



    }

    @SuppressWarnings("SuspiciousNameCombination")
    @Override
    protected void onLayout(boolean changed, int l, int t, int r, int b) {
        if (getChildCount() > 0) {

            int width = r - l;
            int height = b - t;

            int previewWidth = width;
            int previewHeight = height;

            if (previewSize != null) {
                previewWidth = previewSize.width;
                previewHeight = previewSize.height;

                int displayOrientation = ScanUtils.configureCameraAngle((Activity) context);
                if (displayOrientation == 90 || displayOrientation == 270) {
                    previewWidth = previewSize.height;
                    previewHeight = previewSize.width;
                }

                Log.d(TAG, "previewWidth:" + previewWidth + " previewHeight:" + previewHeight);
            }

            int nW;
            int nH;
            int top;
            int left;

            float scale = 1.0f;

            // Center the child SurfaceView within the parent.
            if (width * previewHeight < height * previewWidth) {
                Log.d(TAG, "center horizontally");
                int scaledChildWidth = (int) ((previewWidth * height / previewHeight) * scale);
                nW = (width + scaledChildWidth) / 2;
                nH = (int) (height * scale);
                top = 0;
                left = (width - scaledChildWidth) / 2;
            } else {
                Log.d(TAG, "center vertically");
                int scaledChildHeight = (int) ((previewHeight * width / previewWidth) * scale);
                nW = (int) (width * scale);
                nH = (height + scaledChildHeight) / 2;
                top = (height - scaledChildHeight) / 2;
                left = 0;
            }
            mSurfaceView.layout(left, top, nW, nH);
            scanCanvasView.layout(left, top, nW, nH);

            Log.d("layout", "left:" + left);
            Log.d("layout", "top:" + top);
            Log.d("layout", "right:" + nW);
            Log.d("layout", "bottom:" + nH);
        }
    }

    public boolean cameraFocus(MotionEvent event) {
//        camera.cancelAutoFocus();
//        surfaceChanged(null, 1, 1, 1);
        if (this.camera != null) {
//            Camera camera = this.camera.getCamera();
            camera.cancelAutoFocus();
            Rect focusRect = calculateTapArea(event.getX(), event.getY(), 1f);

            Camera.Parameters parameters = camera.getParameters();
            parameters.setFocusMode(Camera.Parameters.FOCUS_MODE_AUTO);

            if (parameters.getMaxNumFocusAreas() > 0) {
                List<Camera.Area> mylist = new ArrayList<Camera.Area>();
                mylist.add(new Camera.Area(focusRect, 1000));
                parameters.setFocusAreas(mylist);
            }

//            try {
//
//            } catch (Exception e) {
//
//            }


            camera.setParameters(parameters);
            camera.autoFocus(new Camera.AutoFocusCallback() {
                @Override
                public void onAutoFocus(boolean success, Camera camera) {
                    camera.cancelAutoFocus();
                    Camera.Parameters params = camera.getParameters();
                    if (!params.getFocusMode().equals(Camera.Parameters.FOCUS_MODE_CONTINUOUS_PICTURE)) {
                        params.setFocusMode(Camera.Parameters.FOCUS_MODE_CONTINUOUS_PICTURE);
                        camera.setParameters(params);
                    }
                }
            });
            return true;
        }
        return false;
    }
    private int clamp(int x, int min, int max) {
        if (x > max) {
            return max;
        }
        if (x < min) {
            return min;
        }
        return x;
    }


    /**
     * Convert touch position x:y to {@link Camera.Area} position -1000:-1000 to 1000:1000.
     */
    private Matrix matrix;
    private int focusAreaSize = 10;
    private Rect calculateTapArea(float x, float y, float coefficient) {
        if (matrix == null) matrix = new Matrix();
        int areaSize = Float.valueOf(focusAreaSize * coefficient).intValue();

        int left = clamp((int) x - areaSize / 2, 0, previewSize.width - areaSize);
        int top = clamp((int) y - areaSize / 2, 0, previewSize.height - areaSize);

        RectF rectF = new RectF(left, top, left + areaSize, top + areaSize);
        matrix.mapRect(rectF);

        return new Rect(Math.round(rectF.left), Math.round(rectF.top), Math.round(rectF.right), Math.round(rectF.bottom));
    }

    @Override
    public void onAutoFocus(boolean b, Camera camera) {

    }


    public class Line implements Comparable<Line>{
        Double rho;
        Double theta;

        public Line(double rho, double theta) {
            this.rho = rho;
            this.theta = theta;
        }

        public Double getRho() {
            return rho;
        }

        public void setRho(Double rho) {
            this.rho = rho;
        }

        public Double getTheta() {
            return theta;
        }

        public void setTheta(Double theta) {
            this.theta = theta;
        }

        @Override
        public int compareTo(Line o) {
            return this.getRho().compareTo(o.getRho());
        }


        public boolean compare(Line o) {
            return this.getRho() == o.getRho() && this.getTheta() == this.getTheta();
        }
    }

    public  class LinePair {
        Line line1;
        Line line2;

        public LinePair(Line line1, Line line2) {
            this.line1 = line1;
            this.line2 = line2;
        }

        public Line getLine1() {
            return line1;
        }

        public void setLine1(Line line1) {
            this.line1 = line1;
        }

        public Line getLine2() {
            return line2;
        }

        public void setLine2(Line line2) {
            this.line2 = line2;
        }
    }

    public class Intersection {
        int id;
        LinePair linePair;
        Point coords;

        public Intersection(int id, LinePair linePair, Point coords) {
            this.id = id;
            this.linePair = linePair;
            this.coords = coords;
        }

        public int getId() {
            return id;
        }

        public void setId(int id) {
            this.id = id;
        }

        public LinePair getLinePair() {
            return linePair;
        }

        public void setLinePair(LinePair linePair) {
            this.linePair = linePair;
        }

        public Point getCoords() {
            return coords;
        }

        public void setCoords(Point coords) {
            this.coords = coords;
        }
    }
}

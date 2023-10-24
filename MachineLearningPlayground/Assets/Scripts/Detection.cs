using System.Collections;
using System.Collections.Generic;
using System;
using System.Linq;
using System.Data;

using UnityEngine;
using Unity.Barracuda;
using TMPro;

using HoloLensCameraStream;

#if WINDOWS_UWP && XR_PLUGIN_OPENXR
using Windows.Perception.Spatial;
#endif

#if WINDOWS_UWP
using Windows.UI.Input.Spatial;
#endif


public class Detection : MonoBehaviour
{
    public NNModel model;
    private Model runtimeModel;
    private IWorker worker;

    public int cameraResolutionWidth;
    public int cameraResolutionHeight;
    private HoloLensCameraStream.Resolution resolution;
    private VideoCapture videoCapture;

    public int inferenceImgSize = 160;
    public int numClasses = 80;
    public float confidenceThreshold;
    
    private List<DetectionResult> boxes;
    private List<Tuple<GameObject, Renderer>> labels;
    private COCONames names;
    private COCOColors colors;
    private Texture2D pictureTexture;
    private Texture2D croppedTexture;

    private RaycastLaser laser;
    public Material laserMaterial;

    private TextMeshPro textMesh;

    private IntPtr _spatialCoordinateSystemPtr;

    Matrix4x4 camera2WorldMatrix;
    Matrix4x4 projectionMatrix;

    Matrix4x4 camera2WorldMatrix_local;
    Matrix4x4 projectionMatrix_local;

#if WINDOWS_UWP && XR_PLUGIN_OPENXR
    SpatialCoordinateSystem _spatialCoordinateSystem;
#endif

    private byte[] _latestImageBytes;
    private bool stopVideo;

    private class SampleStruct
    {
        public float[] camera2WorldMatrix, projectionMatrix;
        public byte[] data;
    }

    void Start()
    {
#if WINDOWS_UWP


#if XR_PLUGIN_OPENXR

        _spatialCoordinateSystem = Microsoft.MixedReality.OpenXR.PerceptionInterop.GetSceneCoordinateSystem(UnityEngine.Pose.identity) as SpatialCoordinateSystem;

#endif
#endif
        CameraStreamHelper.Instance.GetVideoCaptureAsync(OnVideoCaptureCreated);

        runtimeModel = ModelLoader.Load(model);
        worker = WorkerFactory.CreateWorker(WorkerFactory.Type.ComputePrecompiled, runtimeModel);

        names = new COCONames();
        colors = new COCOColors();

        croppedTexture = new Texture2D(inferenceImgSize, inferenceImgSize, TextureFormat.RGB24, false);

        var _ = GameObject.Find("Prediction");
        textMesh = _.GetComponent<TextMeshPro>();
        boxes = new List<DetectionResult>();
        labels = new List<Tuple<GameObject, Renderer>>();
        laser = GetComponent<RaycastLaser>();

        StartCoroutine(wait());
        StartCoroutine(DetectWebcam());
    }

    private IEnumerator wait()
    {
        yield return new WaitForSeconds(2.0f);
    }

    private void OnDestroy()
    {
        if (videoCapture == null)
            return;

        videoCapture.FrameSampleAcquired += null;
        videoCapture.Dispose();
    }

    private void OnVideoCaptureCreated(VideoCapture v)
    {
        if (v == null)
        {
            Debug.LogError("No VideoCapture found");
            return;
        }

        videoCapture = v;

#if WINDOWS_UWP
#if XR_PLUGIN_OPENXR
        CameraStreamHelper.Instance.SetNativeISpatialCoordinateSystem(_spatialCoordinateSystem);

#endif
#endif

        resolution = CameraStreamHelper.Instance.GetLowestResolution();
        resolution = new HoloLensCameraStream.Resolution(cameraResolutionWidth, cameraResolutionHeight);
        float frameRate = CameraStreamHelper.Instance.GetHighestFrameRate(resolution);

        videoCapture.FrameSampleAcquired += OnFrameSampleAcquired;

        CameraParameters cameraParams = new CameraParameters();
        cameraParams.cameraResolutionHeight = resolution.height;
        cameraParams.cameraResolutionWidth = resolution.width;
        cameraParams.frameRate = Mathf.RoundToInt(frameRate);
        cameraParams.pixelFormat = CapturePixelFormat.BGRA32;

        UnityEngine.WSA.Application.InvokeOnAppThread(() => { pictureTexture = new Texture2D(resolution.width, resolution.height, TextureFormat.BGRA32, false); }, false);

        videoCapture.StartVideoModeAsync(cameraParams, OnVideoModeStarted);

        Debug.LogWarning($"{resolution.height},  {resolution.width}, {cameraParams.frameRate}");
    }

    private void OnVideoModeStarted(VideoCaptureResult result)
    {
        if (result.success == false)
        {
            Debug.LogWarning("Could not start video mode.");
            return;
        }

        Debug.Log("Video capture started.");
    }

    private void OnFrameSampleAcquired(VideoCaptureSample sample)
    {
        // Allocate byteBuffer
        if (_latestImageBytes == null || _latestImageBytes.Length < sample.dataLength)
            _latestImageBytes = new byte[sample.dataLength];

        // Fill frame struct 
        SampleStruct s = new SampleStruct();
        sample.CopyRawImageDataIntoBuffer(_latestImageBytes);
        s.data = _latestImageBytes;

        // Get the cameraToWorldMatrix and projectionMatrix
        if (!sample.TryGetCameraToWorldMatrix(out s.camera2WorldMatrix) || !sample.TryGetProjectionMatrix(out s.projectionMatrix))
            return;

        sample.Dispose();

        camera2WorldMatrix = LocatableCameraUtils.ConvertFloatArrayToMatrix4x4(s.camera2WorldMatrix);
        projectionMatrix = LocatableCameraUtils.ConvertFloatArrayToMatrix4x4(s.projectionMatrix);

        UnityEngine.WSA.Application.InvokeOnAppThread(() =>
        {
            pictureTexture.LoadRawTextureData(s.data);
            pictureTexture.Apply();

            Vector3 inverseNormal = -camera2WorldMatrix.GetColumn(2);
            // Position the canvas object slightly in front of the real world web camera.
            Vector3 imagePosition = camera2WorldMatrix.GetColumn(3) - camera2WorldMatrix.GetColumn(2);

#if XR_PLUGIN_WINDOWSMR || XR_PLUGIN_OPENXR

            Camera unityCamera = Camera.main;
            Matrix4x4 invertZScaleMatrix = Matrix4x4.Scale(new Vector3(1, 1, -1));
            Matrix4x4 localToWorldMatrix = camera2WorldMatrix * invertZScaleMatrix;
            unityCamera.transform.localPosition = localToWorldMatrix.GetColumn(3);
            unityCamera.transform.localRotation = Quaternion.LookRotation(localToWorldMatrix.GetColumn(2), localToWorldMatrix.GetColumn(1));
#endif
        }, false);
    }

    void Update()
    {
        if (textMesh != null)
        {
            textMesh.text = $"{boxes.Count}";
        }
    }

    public IEnumerator DetectWebcam()
    {
        List<DetectionResult> boxes_tmp = new List<DetectionResult>();
        while (true)
        {
            if (pictureTexture)
            {
                camera2WorldMatrix_local = camera2WorldMatrix;
                projectionMatrix_local = projectionMatrix;

                CropTexture(inferenceImgSize, inferenceImgSize);
                var tensor = new Tensor(croppedTexture, false, Vector4.one, Vector4.zero);

                worker.Execute(tensor).FlushSchedule(true);
                Tensor result = worker.PeekOutput("output0");
                
                boxes_tmp.Clear();
                boxes.Clear();

                ParseYoloOutput(result, confidenceThreshold, boxes_tmp);
                boxes = NonMaxSuppression(0.5f, boxes_tmp);

                if (textMesh != null)
                {
                    textMesh.text = $"Boxes: {boxes.Count}";
                }

                foreach (var (go, r) in labels)
                {
                    Destroy(r);
                    Destroy(go);
                }
                labels.Clear();

                foreach (var l in boxes)
                {
                    GenerateBoundingBox(l, camera2WorldMatrix_local, projectionMatrix_local);
                }

                tensor.Dispose();
                result.Dispose();
                yield return null;
            }
            else
            {
                yield return null;
            }
        }
    }
    private void ParseYoloOutput(Tensor tensor, float confidenceThreshold, List<DetectionResult> boxes)
    {
            for (int i = 0; i < tensor.shape.width; i++)
            {
                var (label, confidence) = GetClassIdx(tensor, i, 0);
                if (confidence < confidenceThreshold)
                {
                    continue;
                }

                BoundingBox box = ExtractBoundingBox(tensor, i);
                var labelName = names.map[label];
                boxes.Add(new DetectionResult
                {
                    Bbox = box,
                    Confidence = confidence,
                    Label = labelName,
                    LabelIdx = label
                });
            }
    }

    private BoundingBox ExtractBoundingBox(Tensor tensor, int row)
    {
        return new BoundingBox
        {
            X = tensor[0, 0, row, 0],
            Y = tensor[0, 0, row, 1],
            Width = tensor[0, 0, row, 2],
            Height = tensor[0, 0, row, 3]
        };
    }

    private ValueTuple<int, float> GetClassIdx(Tensor tensor, int row, int batch)
    {
        int classIdx = 0;
        float maxConf = tensor[0, 0, row, 4];
        for (int i = 0; i < numClasses; i++)
        {
            if (tensor[batch, 0, row, 4 + i] > maxConf)
            {
                maxConf = tensor[0, 0, row, 4 + i];
                classIdx = i;
            }
        }

        return (classIdx, maxConf);
    }

    private float IoU(Rect boundingBoxA, Rect boundingBoxB)
    {
        float intersectionArea = Mathf.Max(0, Mathf.Min(boundingBoxA.xMax, boundingBoxB.xMax) - Mathf.Max(boundingBoxA.xMin, boundingBoxB.xMin)) *
                        Mathf.Max(0, Mathf.Min(boundingBoxA.yMax, boundingBoxB.yMax) - Mathf.Max(boundingBoxA.yMin, boundingBoxB.yMin));

        float unionArea = boundingBoxA.width * boundingBoxA.height + boundingBoxB.width * boundingBoxB.height - intersectionArea;

        if (unionArea == 0)
        {
            return 0;
        }

        return intersectionArea / unionArea;
    }

    private List<DetectionResult> NonMaxSuppression(float threshold, List<DetectionResult> boxes)
    {
        var results = new List<DetectionResult>();
        if (boxes.Count == 0)
        {
            return results;
        }
        var detections = boxes.OrderByDescending(b => b.Confidence).ToList();
        results.Add(detections[0]);

        for (int i = 1; i < detections.Count; i++)
        {
            bool add = true;
            for (int j = 0; j < results.Count; j++)
            {
                float iou = IoU(detections[i].Rect, results[j].Rect);
                if (iou > threshold)
                {
                    add = false;
                    break;
                }
            }
            if (add)
                results.Add(detections[i]);
        }

        return results;

    }
    public Vector3 shootLaserFrom(Vector3 from, Vector3 direction, float length, Material mat = null)
    {
        Ray ray = new Ray(from, direction);
        Vector3 to = from + length * direction;

        RaycastHit hit;
        if (Physics.Raycast(ray, out hit, length))
            to = hit.point;

        return to;
    }

    public RaycastHit shootLaserRaycastHit(Vector3 from, Vector3 direction, float length, Material mat = null)
    {
        Ray ray = new Ray(from, direction);
        Vector3 to = from + length * direction;

        RaycastHit hit;
        if (Physics.Raycast(ray, out hit, length))
            to = hit.point;

        return hit;
    }

    public Vector3 GenerateBoundingBox(DetectionResult det, Matrix4x4 camera2WorldMatrix_local, Matrix4x4 projectionMatrix_local)
    {
        var x_offset = (cameraResolutionWidth - inferenceImgSize) / 2;
        var y_offset = (cameraResolutionHeight - inferenceImgSize) / 2;
        Vector3 direction = LocatableCameraUtils.PixelCoordToWorldCoord(camera2WorldMatrix_local, projectionMatrix_local, resolution, new Vector2(det.Bbox.X + x_offset, det.Bbox.Y + y_offset));
        var centerHit = shootLaserRaycastHit(camera2WorldMatrix_local.GetColumn(3), direction, 10f);

        var distance = centerHit.distance;
        distance -= 0.05f;

        Vector3 corner_0 = LocatableCameraUtils.PixelCoordToWorldCoord(camera2WorldMatrix_local, projectionMatrix_local, resolution, new Vector2((det.Bbox.X + x_offset) - (det.Bbox.Width / 2), det.Bbox.Y + y_offset - (det.Bbox.Height / 2)));
        Vector3 corner_1 = LocatableCameraUtils.PixelCoordToWorldCoord(camera2WorldMatrix_local, projectionMatrix_local, resolution, new Vector2((det.Bbox.X + x_offset) - (det.Bbox.Width / 2), det.Bbox.Y + y_offset + (det.Bbox.Height / 2)));
        Vector3 corner_2 = LocatableCameraUtils.PixelCoordToWorldCoord(camera2WorldMatrix_local, projectionMatrix_local, resolution, new Vector2((det.Bbox.X + x_offset) + (det.Bbox.Width / 2), det.Bbox.Y + y_offset - (det.Bbox.Height / 2)));
        Vector3 corner_3 = LocatableCameraUtils.PixelCoordToWorldCoord(camera2WorldMatrix_local, projectionMatrix_local, resolution, new Vector2((det.Bbox.X + x_offset) + (det.Bbox.Width / 2), det.Bbox.Y + y_offset + (det.Bbox.Height / 2)));

        var point_0 = shootLaserFrom(camera2WorldMatrix_local.GetColumn(3), corner_0, distance);
        var point_1 = shootLaserFrom(camera2WorldMatrix_local.GetColumn(3), corner_1, distance);
        var point_2 = shootLaserFrom(camera2WorldMatrix_local.GetColumn(3), corner_2, distance);
        var point_3 = shootLaserFrom(camera2WorldMatrix_local.GetColumn(3), corner_3, distance);

        var go = new GameObject();

        go.transform.position = point_0;
        go.transform.rotation = Camera.main.transform.rotation;

        var renderer = go.GetComponent<Renderer>();

        LineRenderer lr = go.AddComponent<LineRenderer>();
        lr.widthMultiplier = 0.01f;
        lr.loop = true;
        lr.positionCount = 4;
        lr.material = laserMaterial;
        lr.material.color = colors.map[det.LabelIdx];

        lr.SetPosition(0, point_0);
        lr.SetPosition(3, point_1);
        lr.SetPosition(1, point_2);
        lr.SetPosition(2, point_3);

        labels.Add(Tuple.Create(go, renderer));

        return centerHit.point;
    }

    private void CropTexture(int cropWidth, int cropHeight)
    {
        int centerX = pictureTexture.width / 2 - cropWidth / 2;
        int centerY = pictureTexture.height / 2 - cropHeight / 2;
        Color[] pixels = pictureTexture.GetPixels(centerX, centerY, cropWidth, cropHeight);

        croppedTexture.SetPixels(pixels);
        croppedTexture.Apply();
    }
}
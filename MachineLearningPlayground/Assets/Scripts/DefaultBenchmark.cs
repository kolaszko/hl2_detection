using System.Collections;
using System.Collections.Generic;
using System;
using System.Linq;
using System.Data;

using UnityEngine;
using Unity.Barracuda;
using TMPro;

using HoloLensCameraStream;
using Unity.XR.CoreUtils;
using Microsoft.MixedReality.Toolkit.UI;
using static UnityEngine.UI.GridLayoutGroup;
using Unity.VisualScripting;
using Microsoft.MixedReality.Toolkit.Diagnostics;
using System.Threading.Tasks;

#if WINDOWS_UWP && XR_PLUGIN_OPENXR
using Windows.Perception.Spatial;
#endif

#if WINDOWS_UWP
using Windows.UI.Input.Spatial;
#endif


public class DefaultBenchmark : MonoBehaviour
{
    public NNModel model;
    public int numClasses = 80;
    public int batchSize = 1;

    private COCONames names;
    private COCOColors colors;
    private YoloDetector detector;
    public float confidenceThreshold;
    List<DetectionResult> boxes;
    public int imgSize = 224;
    List<Tuple<GameObject, Renderer>> labels;

    private int benchmarkSize = 101;
    private int benchmarkCounter = 0;
    private string benchmarkString = "";

    private GameObject _picture;
    private Renderer _pictureRenderer;
    private Texture2D _pictureTexture;

    private RaycastLaser _laser;
    public Material _laserMaterial;

    private TextMeshPro textMesh;

    public int resolutionWidth;
    public int resolutionHeight;
    private HoloLensCameraStream.Resolution _resolution;
    private VideoCapture _videoCapture;

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

#if WINDOWS_UWP
    private SpatialInteractionManager _spatialInteraction;
#endif

    private class SampleStruct
    {
        public float[] camera2WorldMatrix, projectionMatrix;
        public byte[] data;
    }

    void Awake()
    {
#if WINDOWS_UWP
        UnityEngine.WSA.Application.InvokeOnUIThread(() =>
        {
            _spatialInteraction = SpatialInteractionManager.GetForCurrentView();
        }, true);
        _spatialInteraction.SourcePressed += SpatialInteraction_SourcePressed;
#endif
    }

    void Start()
    {
#if WINDOWS_UWP

#if XR_PLUGIN_WINDOWSMR

        _spatialCoordinateSystemPtr = UnityEngine.XR.WindowsMR.WindowsMREnvironment.OriginSpatialCoordinateSystem;

#elif XR_PLUGIN_OPENXR

        _spatialCoordinateSystem = Microsoft.MixedReality.OpenXR.PerceptionInterop.GetSceneCoordinateSystem(UnityEngine.Pose.identity) as SpatialCoordinateSystem;

#elif BUILTIN_XR

#if UNITY_2017_2_OR_NEWER
        _spatialCoordinateSystemPtr = UnityEngine.XR.WSA.WorldManager.GetNativeISpatialCoordinateSystemPtr();
#else
        _spatialCoordinateSystemPtr = UnityEngine.VR.WSA.WorldManager.GetNativeISpatialCoordinateSystemPtr();
#endif

#endif

#endif

        CameraStreamHelper.Instance.GetVideoCaptureAsync(OnVideoCaptureCreated);
        names = new COCONames();
        colors = new COCOColors();
        detector = new YoloDetector(model);
        var _ = GameObject.Find("Prediction");
        textMesh = _.GetComponent<TextMeshPro>();
        boxes = new List<DetectionResult>();
        labels = new List<Tuple<GameObject, Renderer>>();

        _laser = GetComponent<RaycastLaser>();

        StartCoroutine(wait());
        StartCoroutine(DetectWebcam());
    }

    private IEnumerator wait()
    {
        yield return new WaitForSeconds(5.0f);
    }

    private void OnDestroy()
    {
        if (_videoCapture == null)
            return;

        _videoCapture.FrameSampleAcquired += null;
        _videoCapture.Dispose();

#if WINDOWS_UWP
        _spatialInteraction.SourcePressed -= SpatialInteraction_SourcePressed;
        _spatialInteraction = null;
#endif
    }

    private void OnVideoCaptureCreated(VideoCapture v)
    {
        if (v == null)
        {
            Debug.LogError("No VideoCapture found");
            return;
        }

        _videoCapture = v;

        //Request the spatial coordinate ptr if you want fetch the camera and set it if you need to 
#if WINDOWS_UWP

#if XR_PLUGIN_OPENXR
        CameraStreamHelper.Instance.SetNativeISpatialCoordinateSystem(_spatialCoordinateSystem);
#elif XR_PLUGIN_WINDOWSMR || BUILTIN_XR
        CameraStreamHelper.Instance.SetNativeISpatialCoordinateSystemPtr(_spatialCoordinateSystemPtr);
#endif

#endif

        _resolution = CameraStreamHelper.Instance.GetLowestResolution();
        _resolution = new HoloLensCameraStream.Resolution(resolutionWidth, resolutionHeight);
        float frameRate = CameraStreamHelper.Instance.GetHighestFrameRate(_resolution);

        _videoCapture.FrameSampleAcquired += OnFrameSampleAcquired;

        CameraParameters cameraParams = new CameraParameters();
        cameraParams.cameraResolutionHeight = _resolution.height;
        cameraParams.cameraResolutionWidth = _resolution.width;
        cameraParams.frameRate = Mathf.RoundToInt(frameRate);
        cameraParams.pixelFormat = CapturePixelFormat.BGRA32;

        UnityEngine.WSA.Application.InvokeOnAppThread(() => { _pictureTexture = new Texture2D(_resolution.width, _resolution.height, TextureFormat.BGRA32, false); }, false);

        _videoCapture.StartVideoModeAsync(cameraParams, OnVideoModeStarted);

        Debug.LogWarning($"{_resolution.height},  {_resolution.width}, {cameraParams.frameRate}");
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
            _pictureTexture.LoadRawTextureData(s.data);
            _pictureTexture.Apply();

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

#if WINDOWS_UWP
    private void SpatialInteraction_SourcePressed(SpatialInteractionManager sender, SpatialInteractionSourceEventArgs args)
    {
        var item = args.State;
        UnityEngine.WSA.Application.InvokeOnAppThread(() =>
        {
            Debug.Log("SourcePressed");

            for (int i = _laser.transform.childCount - 1; i >= 0; --i)
            {
                GameObject.DestroyImmediate(_laser.transform.GetChild(i).gameObject);
            }

            stopVideo = !stopVideo;

        }, false);
    }
#endif

    void Update()
    {
        if (textMesh != null)
        {
            textMesh.text = $"{boxes.Count}";
        }
    }

    public IEnumerator DetectWebcam()
    {
        Debug.Log("Staring detection coroutine!");
        List<DetectionResult> boxes_tmp = new List<DetectionResult>();
        while (true)
        {
            if (_pictureTexture)
            {
                camera2WorldMatrix_local = camera2WorldMatrix;
                projectionMatrix_local = projectionMatrix;

                var sw_pre = System.Diagnostics.Stopwatch.StartNew();
                textMesh.text = System.Diagnostics.Stopwatch.IsHighResolution.ToString();
                var data = _pictureTexture.GetPixels32();
                Tensor tensor = ToTensor(data, imgSize, imgSize, 424);
                sw_pre.Stop();
                var pre_time = sw_pre.Elapsed;

                var sw_inference = System.Diagnostics.Stopwatch.StartNew();
                detector.worker.Execute(tensor).FlushSchedule(true);
                Tensor result = detector.worker.PeekOutput("output0");
                sw_inference.Stop();
                var inference_time = sw_inference.Elapsed;
                boxes_tmp.Clear();
                boxes.Clear();

                var sw_post = System.Diagnostics.Stopwatch.StartNew();
                ParseYoloOutput(result, confidenceThreshold, boxes_tmp);
                boxes = NonMaxSuppression(0.5f, boxes_tmp);
                sw_post.Stop();
                var post_time = sw_post.Elapsed;

                benchmarkString += $"{pre_time.TotalMilliseconds},{inference_time.TotalMilliseconds},{post_time.TotalMilliseconds} \r\n";
                benchmarkCounter++;

                Debug.LogWarning($"<debug> {boxes.Count}");

                if (benchmarkCounter == benchmarkSize){
                    SaveData();
                    textMesh.text = "Benchmark Saved!";
                }

                if (textMesh != null)
                {
                    textMesh.text = $"Boxes: {boxes.Count}";
                }

                tensor.Dispose();

                foreach (var (go, r) in labels)
                {
                    Destroy(r);
                    Destroy(go);
                }
                labels.Clear();

                foreach (var l in boxes)
                {
                    GenerateBoundingBox(l, camera2WorldMatrix_local, projectionMatrix);
                }

                yield return null;
            }
            else
            {
                yield return null;
            }
        }
    }
    private static Tensor ToTensor(Color32[] pic, int width, int height, int requestedWidth)
    {
        float[] floatValues = new float[width * height * 3];

        int beginning = (((pic.Length / requestedWidth) - height) * requestedWidth) / 2;
        int leftOffset = (requestedWidth - width) / 2;

        Span<float> floatValuesSpan = floatValues.AsSpan();
        Span<Color32> picSpan = pic.AsSpan(beginning + leftOffset);

        for (int i = 0; i < height; i++)
        {
            Span<float> rowSpan = floatValuesSpan.Slice(i * width * 3, width * 3);
            Span<Color32> rowDataSpan = picSpan.Slice(i * requestedWidth, width);

            for (int j = 0; j < width; j++)
            {
                var color = rowDataSpan[j];

                rowSpan[j * 3 + 0] = (color.r - 0.0f) / 255.0f;
                rowSpan[j * 3 + 1] = (color.g - 0.0f) / 255.0f;
                rowSpan[j * 3 + 2] = (color.b - 0.0f) / 255.0f;
            }
        }

        return new Tensor(1, height, width, 3, floatValues);
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
        var sortedBoxes = boxes.OrderByDescending(b => b.Confidence).ToList();
        var results = new List<DetectionResult>();

        for (int i = 0; i < sortedBoxes.Count; i++)
        {
            var boxA = sortedBoxes[i];
            bool discard = false;

            for (var j = i + 1; j < sortedBoxes.Count; j++)
            {
                var boxB = sortedBoxes[j];

                if (IoU(boxA.Rect, boxB.Rect) > threshold)
                {
                    discard = true;
                    break;
                }
            }

            if (!discard)
            {
                results.Add(boxA);
            }
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
        var x_offset = (resolutionWidth - imgSize) / 2;
        var y_offset = (resolutionHeight - imgSize) / 2;
        Vector3 direction = LocatableCameraUtils.PixelCoordToWorldCoord(camera2WorldMatrix_local, projectionMatrix_local, _resolution, new Vector2(det.Bbox.X + x_offset, det.Bbox.Y + y_offset));
        var centerHit = shootLaserRaycastHit(camera2WorldMatrix_local.GetColumn(3), direction, 10f);

        var distance = centerHit.distance;
        distance -= 0.05f;

        Vector3 corner_0 = LocatableCameraUtils.PixelCoordToWorldCoord(camera2WorldMatrix_local, projectionMatrix_local, _resolution, new Vector2((det.Bbox.X + x_offset) - (det.Bbox.Width / 2), det.Bbox.Y + y_offset - (det.Bbox.Height / 2)));
        Vector3 corner_1 = LocatableCameraUtils.PixelCoordToWorldCoord(camera2WorldMatrix_local, projectionMatrix_local, _resolution, new Vector2((det.Bbox.X + x_offset) - (det.Bbox.Width / 2), det.Bbox.Y + y_offset + (det.Bbox.Height / 2)));
        Vector3 corner_2 = LocatableCameraUtils.PixelCoordToWorldCoord(camera2WorldMatrix_local, projectionMatrix_local, _resolution, new Vector2((det.Bbox.X + x_offset) + (det.Bbox.Width / 2), det.Bbox.Y + y_offset - (det.Bbox.Height / 2)));
        Vector3 corner_3 = LocatableCameraUtils.PixelCoordToWorldCoord(camera2WorldMatrix_local, projectionMatrix_local, _resolution, new Vector2((det.Bbox.X + x_offset) + (det.Bbox.Width / 2), det.Bbox.Y + y_offset + (det.Bbox.Height / 2)));

        var point_0 = shootLaserFrom(camera2WorldMatrix_local.GetColumn(3), corner_0, distance);
        var point_1 = shootLaserFrom(camera2WorldMatrix_local.GetColumn(3), corner_1, distance);
        var point_2 = shootLaserFrom(camera2WorldMatrix_local.GetColumn(3), corner_2, distance);
        var point_3 = shootLaserFrom(camera2WorldMatrix_local.GetColumn(3), corner_3, distance);

        var go = new GameObject();
        Vector3 inverseNormal = -camera2WorldMatrix_local.GetColumn(2);

        go.transform.position = point_0;
        go.transform.rotation = Camera.main.transform.rotation;

        var renderer = go.GetComponent<Renderer>();

        LineRenderer lr = go.AddComponent<LineRenderer>();
        lr.widthMultiplier = 0.01f;
        lr.loop = true;
        lr.positionCount = 4;
        lr.material = _laserMaterial;
        lr.material.color = colors.map[det.LabelIdx];

        lr.SetPosition(0, point_0);
        lr.SetPosition(3, point_1);
        lr.SetPosition(1, point_2);
        lr.SetPosition(2, point_3);

/*        var text = go.GetOrAddComponent<ToolTip>();
        text.Anchor = go;
        text.ToolTipText = $"{det.Label} : {det.Confidence:N2}";
        text.FontSize = 20;
        text.ContentScale = 3;
        text.Pivot.transform.rotation = go.transform.rotation;
        text.Pivot.transform.position = go.transform.position + Vector3.up * 0.05f;*/

        labels.Add(Tuple.Create(go, renderer));

        return centerHit.point;
    }

    private async void SaveData()
    {
#if WINDOWS_UWP
        var storageFolder = Windows.Storage.ApplicationData.Current.LocalFolder;
        Windows.Storage.StorageFile posFile = await storageFolder.CreateFileAsync("defaultBenchmarkResults.csv", Windows.Storage.CreationCollisionOption.ReplaceExisting);
        Task task = new Task(
        async () =>
            {
                await Windows.Storage.FileIO.AppendTextAsync(posFile, benchmarkString);
            }
        );
        task.Start();
        task.Wait();
#endif
    }
}

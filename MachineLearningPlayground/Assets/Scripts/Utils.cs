using System.Collections.Generic;
using UnityEngine;
using System;


public class BoundingBox
{
    public float X { get; set; }
    public float Y { get; set; }
    public float Width { get; set; }
    public float Height { get; set; }
}

public class DetectionResult
{
    public BoundingBox Bbox { get; set; }
    public string Label { get; set; }
    public int LabelIdx { get; set; }
    public float Confidence { get; set; }

    public Rect Rect
    {
        get { return new Rect(Bbox.X, Bbox.Y, Bbox.Width, Bbox.Height); }
    }

    public override string ToString()
    {
        return $"{Label}:{Confidence}";
    }
}

public class COCONames
{
    public List<String> map;

    public COCONames()
    {
        map = new List<string>(){
    "person",
    "bicycle",
    "car",
    "motorcycle",
    "airplane",
    "bus",
    "train",
    "truck",
    "boat",
    "traffic light",
    "fire hydrant",
    "stop sign",
    "parking meter",
    "bench",
    "bird",
    "cat",
    "dog",
    "horse",
    "sheep",
    "cow",
    "elephant",
    "bear",
    "zebra",
    "giraffe",
    "backpack",
    "umbrella",
    "handbag",
    "tie",
    "suitcase",
    "frisbee",
    "skis",
    "snowboard",
    "sports ball",
    "kite",
    "baseball bat",
    "baseball glove",
    "skateboard",
    "surfboard",
    "tennis racket",
    "bottle",
    "wine glass",
    "cup",
    "fork",
    "knife",
    "spoon",
    "bowl",
    "banana",
    "apple",
    "sandwich",
    "orange",
    "broccoli",
    "carrot",
    "hot dog",
    "pizza",
    "donut",
    "cake",
    "chair",
    "couch",
    "potted plant",
    "bed",
    "dining table",
    "toilet",
    "tv",
    "laptop",
    "mouse",
    "remote",
    "keyboard",
    "cell phone",
    "microwave",
    "oven",
    "toaster",
    "sink",
    "refrigerator",
    "book",
    "clock",
    "vase",
    "scissors",
    "teddy bear",
    "hair drier",
    "toothbrush"
};

    }

}

public class COCOColors
{
    public List<Color> map;

    public COCOColors()
    {
        map = new List<Color>();
        for (var i = 0; i < 80; ++i)
        {
            map.Add(UnityEngine.Random.ColorHSV(0f, 1f, 1f, 1f, 0.5f, 1f));
        }
        map[0] = new Color(255.0f, 0.0f, 127.0f);
    }
}


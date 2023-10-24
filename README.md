# hl2_detection

## Configure a project

Steps to run app on HoloLens 2:
1. Clone the repository
2. Checkout to *alpha* branch. This branch uses a [Barracuda](https://docs.unity3d.com/Packages/com.unity.barracuda@3.0/manual/index.html) library, which will be replaced with [Sentis](https://docs.unity3d.com/Packages/com.unity.sentis@1.2/manual/index.html).
3. Open Unity Hub, then **Open**->**Add project from disk** and open a MachineLearningPlayground directory inside this repository.
4. There should a *MRTK Project Configurator* window appear. Select **Unity OpenXR plugin (recommended)** option, the project will configure.
5. The *Settings* window will appear. You can close it, same to a *MRTK Project Configurator* window.
6. In a Unity Editor topbar select **File**->**Build Settings...** . In a *Platform* select a **Universal Windows Platform**, on the right change architecture to **ARM 64** and click on **Switch platform** button on the bottom. Wait for a project to reconfigure.
7. In a Unity Editor in a Project Explorer view (bottom) select the *Assets*, and then *Scenes* directory. Open *SampleScene.unity* file. The scene will load.
8. From a *Hierarchy* panel select *MixedRealityPlayspace*->*Main Camera* object. In a bottom of *Inspector* panel (right side of editor) there is a **Detection** script. The model property is missing, since you have to provide a model by yourself. Follow the next steps to prepare a YOLOv8 model. 

## Prepare YOLOv8 model
Install the ultralytics repo according to manual in: https://github.com/ultralytics/ultralytics . Then, export a weights of YOLOv8 Nano model to .onnx format with opset level of 9. In the following example we use an image input size of 160 by 160 pixels. 

Export script:
```bash
yolo export model=yolov8n.pt format=onnx opset=9 imgsz=160
```

As a result, the yolov8n.onnx was generated in a working directory.

You can also use [this notebook](https://colab.research.google.com/drive/11M6M00Ss0cPvgJOzwaUdw4DUlOhUvdhj?usp=sharing) to export and download a model.  

When exported, copy a .onnx file to a *Assets/Weights** directory. 

## Build and deploy on HL2

1. Drag and drop a exported neural network weights file to a *Model* property in Unity Editor.
2. In top bar select **File**->**Build Settings**->**Build**. Create a folder to place a build files and start a build process.
3. After finished, open a build directory, open a .sln file with a project, make sure to pick a proper build configuration, architecture and device, then build and deploy solution.

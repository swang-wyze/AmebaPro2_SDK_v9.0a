# YOLOv3 Object Detection Demo Quick Start

This demo will demonsrate how to deploy a pre-trained YOLOv3 model on AmebaPro2 to do object detection.

## Configure the NN demo

### Select camera sensor 

Check your camera sensor model, and define it in `<AmebaPro2_SDK>/project/realtek_amebapro2_v0_example/inc/platform_opts.h`

```
//SENSOR_GC2053, SENSOR_PS5258
#define USE_SENSOR SENSOR_PS5258
```

### Enable NN example

The NN example is part of our multi-media video example. Enable the MMFv2 example in `<AmebaPro2_SDK>/project/realtek_amebapro2_v0_example/inc/platform_opts.`
```
/* For MMFv2 example */
#define CONFIG_EXAMPLE_MEDIA_FRAMEWORK  1
#if (defined(CONFIG_EXAMPLE_MEDIA_FRAMEWORK) && CONFIG_EXAMPLE_MEDIA_FRAMEWORK)
#define CONFIG_EXAMPLE_MEDIA_UVCD   0
#define CONFIG_EXAMPLE_MEDIA_VIDEO  1
#define CONFIG_EXAMPLE_MEDIA_CLOUD  0
#define CONFIG_EXAMPLE_MEDIA_NN     0
#define FATFS_DISK_SD 	1
#endif
```
go to `<AmebaPro2_SDK>/project/realtek_amebapro2_v0_example/src/mmfv2_video_example/video_example_media_framework.c` to uncomment the following example
```
// RGB -> NN inference
// H264 -> RTSP (V)
mmf2_example_vipnn_rtsp_init();
```

### Build image

To build the example run the following command:
```
cd <AmebaPro2_SDK>/project/realtek_amebapro2_v0_example/GCC-RELEASE
mkdir build
cd build
cmake .. -G"Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../toolchain.cmake -DBUILD_VIPNN=ON
cmake --build . --target flash
```
The image is located in `<AmebaPro2_SDK>/project/realtek_amebapro2_v0_example/GCC-RELEASE/build/flash_ntz.bin`

Make sure your AmebaPro2 is connected and powered on. Use the Realtek image tool to flash image.

Note: see chapter5 of  [AN0700 Realtek AmebaPro2 application note.en.pdf](https://github.com/HungTseLee/AmebaPro2_SDK/blob/main/doc/AN0700%20Realtek%20AmebaPro2%20application%20note.en.pdf) to use the image tool

## Prepare YOLOv3 model

### Get model NBG file

If you have a pre-trained YOLO model(.cfg & .weights), it can be converted to an NBG file(.nb) by Acuity tool.

### Convert model NBG file to bin file

Now, we can use **model_convert.exe** (in AmebaPro2_SDK/tools) to convert YOLO model from NBG file(.nb) to a binary file(.bin)

```
model_convert.exe model_name.nb model_name.bin
```

Note: If you don't have a pre-trained YOLO model, you can use the model given in SDK to validate the demo.
- `<AmebaPro2_SDK>/project/realtek_amebapro2_v0_example/src/nn_verify/yolov3_tiny.bin`

### Download the model bin to Amebapro2

use the Image tool to download the model bin to the flash with offest **0x300000**

Trigger AmebaPro2 into download mode and follow the step in figure

<p align="center"> <img src="https://user-images.githubusercontent.com/56305789/136130368-9d3aac7d-65f1-4691-8b8f-a244710a5dad.png" alt="test image size" height="50%" width="50%"></p>


## Validation

### Configure WiFi connection and check log

Reboot your device and check the logs.

While runnung the example, you may need to configure WiFi connection by using these commands in uart terminal.

```
ATW0=<WiFi_SSID> : Set the WiFi AP to be connected
ATW1=<WiFi_Password> : Set the WiFi AP password
ATWC : Initiate the connection
```

If everything works fine, you should see the following logs

```
...
[VOE]RGB3 640x480 1/5
[VOE]Start Mem Used ISP/ENC:     0 KB/    0 KB Free=  701
hal_rtl_sys_get_clk 2
GCChipRev data = 8020
GCChipDate data = 20190925
queue 20121bd8 queue mutex 71691380
npu gck vip_drv_init, video memory heap base: 0x71B00000, size: 0x01300000
yuv in 0x714cee00
[VOE][process_rgb_yonly_irq][371]Errrgb ddr frame count overflow : int_status 0x00000008 buf_status 0x00000010 time 15573511 cnt 0
input 0 dim 640 352 3 1, data format=2, quant_format=2, scale=0.003635, zero_point=0
ouput 0 dim 20 11 18 1, data format=2, scale=0.085916, zero_point=161
ouput 1 dim 40 22 18 1, data format=2, scale=0.083689, zero_point=159
---------------------------------
input count 1, output count 2
input param 0
        data_format  2
        memory_type  0
        num_of_dims  4
        quant_format 2
        quant_data  , scale=0.003635, zero_point=0
        sizes        280 160 3 1 0 0
output param 0
        data_format  2
        memory_type  0
        num_of_dims  4
        quant_format 2
        quant_data  , scale=0.085916, zero_point=161
        sizes        14 b 12 1 0 0
output param 1
        data_format  2
        memory_type  0
        num_of_dims  4
        quant_format 2
        quant_data  , scale=0.083689, zero_point=159
        sizes        28 16 12 1 0 0
---------------------------------
in 0, size 640 352
VIPNN opened
siso_array_vipnn started
nn tick[0] = 213
object num = 0
RGB release 0x714cee00
yuv in 0x714cee00
nn tick[0] = 223
object num = 0
RGB release 0x714cee00
yuv in 0x714cee00
nn tick[0] = 222
object num = 0
RGB release 0x714cee00
yuv in 0x714cee00
nn tick[0] = 219
object num = 0
RGB release 0x714cee00
yuv in 0x714cee00
nn tick[0] = 216
object num = 0
...

```

### Use VLC to validate the result

Then, open VLC and create a network stream with URL: rtsp://192.168.x.xx:554

If everything works fine, you should see the object detection result on VLC player.

<p align="center"> <img src="https://user-images.githubusercontent.com/56305789/136002827-1317982e-cec6-4e30-9aa7-45e4ad873407.JPG" alt="test image size" height="80%" width="80%"></p>

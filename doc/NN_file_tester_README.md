# NN Tester with Data in SD Card - Quick Start

This example can read the image set from SD card and save the NN inference result back to SD card.

The JSON file will be saved with object detection results, and the raw output tensor will also be saved.

## Prepare data set and SD card

### Prepare image with bitmap 24bit(8UC3) format and same size

Convert your image data to **24bit .bmp** format with **same size** (416x416 or 640x480...).  

The image should be named with consistent prefix, like `image-0001.bmp, image-0002.bmp, image-0003.bmp ...`, since the fileloader will read them in order.

Then, configure the input image size in `component/example/media_framework/mmf2_example_file_vipnn_tester.c`  
```
#define TEST_IMAGE_WIDTH	416   /* Fix me */
#define TEST_IMAGE_HEIGHT	416   /* Fix me */
```

### Put data to SD card with specified folder path

We assume user's data will be placed in folder "test_bmp/", and the output path is "yolo_result/".  
The **input path**, **number of test image** and **output path** can also be configured in `mmf2_example_file_vipnn_tester.c`  
```
#define TEST_FILE_PATH_PREFIX   "test_bmp/image"            /* Fix me */
#define TEST_FILE_NUM           10                          /* Fix me */

#define FILE_OUT_PATH_PREFIX    "yolo_result/yolo_result"   /* Fix me */
```

Create two folder with name "test_bmp/" and "yolo_result/" in SD card, then put your test images to `SD:/test_bmp/`

**Note:**  
the files in SD card may look like  
```
--- test_bmp/
 |   |--image-0001.bmp
 |   |--image-0002.bmp
 |   |--image-0003.bmp
 |
 -- yolo_result/
     |--yolo_result_1.json
     |--yolo_result_out_tensor0_uint8_1.bin
     |--yolo_result_out_tensor1_uint8_1.bin
     |--yolo_result_2.json
     |--yolo_result_out_tensor0_uint8_2.bin
     |--yolo_result_out_tensor1_uint8_2.bin
     |--yolo_result_3.json
     |--yolo_result_out_tensor0_uint8_3.bin
     |--yolo_result_out_tensor1_uint8_3.bin
```

### Enable NN tester example

The example is part of our multi-media example. Enable the MMFv2 example in `<AmebaPro2_SDK>/project/realtek_amebapro2_v0_example/inc/platform_opts.`
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
// NN image from SD card
mmf2_example_file_vipnn_tester();
```
This demo don't need wifi, so you can disable the wifi connection check in `video_example_media_framework.c`  
```
//wifi_common_init();
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

### Check log

Reboot your device and check the logs.

If everything works fine, you should see the following logs

```
...
Read 519222 bytes.

start offset of data: 54
data_size: 519168

bmp file info: w:416, h:416, Bottom2Top
input 0 dim 416 416 3 1, data format=2, quant_format=2, scale=0.003660, zero_point=0
ouput 0 dim 13 13 255 1, data format=2, scale=0.092055, zero_point=216
ouput 1 dim 26 26 255 1, data format=2, scale=0.093103, zero_point=216
---------------------------------
input count 1, output count 2
input param 0
        data_format  2
        memory_type  0
        num_of_dims  4
        quant_format 2
        quant_data  , scale=0.003660, zero_point=0
        sizes        1a0 1a0 3 1 0 0
output param 0
        data_format  2
        memory_type  0
        num_of_dims  4
        quant_format 2
        quant_data  , scale=0.092055, zero_point=216
        sizes        d d ff 1 0 0
output param 1
        data_format  2
        memory_type  0
        num_of_dims  4
        quant_format 2
        quant_data  , scale=0.093103, zero_point=216
        sizes        1a 1a ff 1 0 0
---------------------------------
in 0, size 416 416
VIPNN opened
filesaver opened
siso_fileloader_vipnn started
siso_vipnn_filesaver started
nn tick[0] = 47
object num = 3
0,c16:63 156 211 379
1,c2:252 60 370 122
2,c7:286 66 340 114

file_size: 519222 bytes.

file name: yolo_result/yolo_result_1.jsonRead 519222 bytes.

Write 582 bytes.
close file (yolo_result/yolo_result_1.json) done.
start offset of data: 54

data_size: 519168

bmp file info: w:416, h:416, Bottom2Top

file name: yolo_result/yolo_result_out_tensor0_uint8_1.bin
Write 43095 bytes.
close file (yolo_result/yolo_result_out_tensor0_uint8_1.bin) done.

file name: yolo_result/yolo_result_out_tensor1_uint8_1.bin
Write 172380 bytes.
close file (yolo_result/yolo_result_out_tensor1_uint8_1.bin) done.
...
...

```

### Validate the result

The yolo object detection result will be stored with json format (yolo_result_4.json):
```
{
	"frame_id":	4,
	"filename":	"yolo_result/yolo_result_4.json",
	"objects":	[{
			"class_id":	0,
			"name":	"person",
			"relative_coordinates":	{
				"top_x":	117,
				"top_y":	87,
				"bottom_x":	181,
				"bottom_y":	381
			},
			"probability":	0.92862808704376221
		}, {
			"class_id":	18,
			"name":	"sheep",
			"relative_coordinates":	{
				"top_x":	258,
				"top_y":	118,
				"bottom_x":	393,
				"bottom_y":	341
			},
			"probability":	0.6933828592300415
		}, {
			"class_id":	16,
			"name":	"dog",
			"relative_coordinates":	{
				"top_x":	70,
				"top_y":	271,
				"bottom_x":	111,
				"bottom_y":	341
			},
			"probability":	0.593533456325531
		}]
}
```

The NN network ouput raw tensor will also be stored as binary file, user can use it to test their customized post-processing algorithm.
```
SD:/yolo_result/yolo_result_out_tensor0_uint8_1.bin
SD:/yolo_result/yolo_result_out_tensor1_uint8_1.bin
...
SD:/yolo_result/yolo_result_out_tensor0_uint8_10.bin
SD:/yolo_result/yolo_result_out_tensor1_uint8_10.bin
...
``` 

Bounding boxes can be drawn according to the .json file or raw ouput tensor on PC:  

<p align="center"> <img src="https://user-images.githubusercontent.com/56305789/140274690-2ca4482c-f0e4-4f9c-b39b-0db3f19fc7b3.png" alt="test image size" height="50%" width="50%"></p>

**Note:**  
If the network is quantized, the zero point, scale or fixed point are required to do post-processing with raw quantized output tensor.  
ex: network quntized with uint8:  
```
ouput 0 dim 13 13 255 1, data format=2, scale=0.092055, zero_point=216
ouput 1 dim 26 26 255 1, data format=2, scale=0.093103, zero_point=216
```
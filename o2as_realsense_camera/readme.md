# Introduction

realsense camera node. 

get depth image and color image from realsense camera with specified serial_number.

this package also provide function that convert captured 
depth image to point cloud data format acceptable for 
omron cad matching library and save it into binary file.

# Usage

1. call connect service
2. call save_frame_for_cad_matching service

save_frame_for_cad_matching service requires two argments named
pcloud_filename and image_filename.
point cloud data is saved to the file with name fcloud_filename
color image data is saved to the file with name image_filename

# ToDo

- Add service to get color image
- Add service to get depth image 
- Add service to get point cloud

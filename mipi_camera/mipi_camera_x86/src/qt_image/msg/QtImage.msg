#image layout
uint32 height
uint32 width
uint32 channel

#image data
uint8[] serialize_image

#box num
uint32 box_n

#box left_up_point
uint32[] box_point1_x
uint32[] box_point1_y

#box_right_down_point
uint32[] box_len_x
uint32[] box_len_y

#box_depth
float32[] box_depth

#box_index
uint8[] box_index
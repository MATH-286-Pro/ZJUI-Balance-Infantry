# 宇树 A1 电机使用说明

`motor_send_t MotorA1_send_left;`          左腿一号电机数据体  
`motor_send_t MotorA1_send_right; `        右腿一号电机数据体  
  
`motor_recv_t Date_left;`         左腿电机接收数据体  
`motor_recv_t MotorA1_recv_left_id00;`    左腿00号电机接收数据体  
`motor_recv_t MotorA1_recv_left_id01;`    左腿01号电机接收数据体  
`motor_recv_t MotorA1_recv_left_id02;`    左腿02号电机接收数据体  



通过命令 `modify_cmd(&MotorA1_send_left, 电机ID, 位置POS, KP, KW)` 
 - 修改 `MotorA1_send_left` 的`ID,POS,KP,KW`数据  
  
  
通过命令 `unitreeA1_rxtx(&huart1)` 
- 使用 `uart1` 串口发送 `MotorA1_send_left` 
- 接收电机数据，并根据ID直接将数据存储到 `MotorA1_recv_left_id00` `MotorA1_recv_left_id01` `MotorA1_recv_left_id02`


## 控制命令说明
`modfiy_pos_cmd(电机结构体，ID，减速后的角度)`

## 结构体说明
`MotorA1_recv_left_id00.Pos` 减速后的角度

## 电机参数
质量 = 605g
位置，速度，力矩 =+ 逆时针旋转
位置，速度，力矩 =- 顺时针旋转

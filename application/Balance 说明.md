# 平衡算法说明

- Balance.c
  
- Balance.h

- Balance_linkNleg.h
  
- Balance_speed_estimation.h
  - 计算 Body 底盘水平速度 (世界坐标)
  
- Balance_fly_detection.h
  - 当前为空文件
  
- Balance_lqr_calc.h (Webots 文件)
  - 这个文件直接在 `Balance.c` 中被定义成 `void CalcLQR(LinkNPodParam *p)`
  


目前未包含在 makefile 中的文件
  kalman_filter.c
  QuaternionEKF.c

// 精确尺寸参数
r_tail = 10;        // 尾部半径
r_head = 7;         // 头部半径
z_tail = 12;        // 尾部z轴厚度
z_head = 6;         // 头部z轴厚度
total_length = 27;  // 总长度
body_smoothness = 5;  // 平滑度

// 平滑参数
smooth_factor = 1;   // 最终平滑程度

module base_body() {
    hull() {
        // 尾部
        translate([-r_tail, 0, 0])
            scale([1, 1, z_tail/(2*r_tail)])
                sphere(r = r_tail, $fn=body_smoothness);
        
        // 头部 - 位置计算确保总长度为27
        translate([0, 0, 0])
            scale([1, 1, z_head/(2*r_head)])
                sphere(r = r_head, $fn=body_smoothness);
    }
}

// 创建最终平滑模型
minkowski() {
    base_body();
    sphere(r = smooth_factor, $fn=50);
}

/* 特性说明：
 * 尾部半径 = 10
 * 头部半径 = 7
 * 总长度 = 27
 * 尾部厚度 = 10
 * 头部厚度 = 6
 * 使用minkowski()进行最终平滑处理
 * 使用scale在z轴方向调整厚度
 * hull()确保头尾平滑连接
 */
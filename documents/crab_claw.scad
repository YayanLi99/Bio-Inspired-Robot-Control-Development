// 基础参数
height = 10;           // 总高度
top_radius = 15;       // 顶面外切圆半径
$fn = 100;             // 平滑度

// 创建单个pyramid
module single_pyramid() {
    // 定义顶点
    points = [
        // 底点
        [0, 0, height],         // 0: 底面点
        
        // 顶面的四个顶点（调整后使一个侧面垂直于顶面）
        [-top_radius, -top_radius, 0], // 1: 左后
        [top_radius, -top_radius, 0],  // 2: 右后
        [top_radius, 0, 0],           // 3: 右前（y坐标为0使侧面垂直）
        [-top_radius, 0, 0]           // 4: 左前（y坐标为0使侧面垂直）
    ];
    
    // 定义面
    faces = [
        [4, 3, 2, 1],    // 顶面（正方形）
        [0, 1, 2],       // 后侧面（三角形）
        [0, 2, 3],       // 右侧面（三角形）
        [0, 3, 4],       // 前侧面（三角形，垂直于顶面）
        [0, 4, 1]        // 左侧面（三角形）
    ];
    
    // 创建多面体
    polyhedron(
        points = points,
        faces = faces,
        convexity = 10
    );
}

// 创建完整的双锥体
module double_pyramid() {
    // 第一个pyramid
    translate([0, 10, 0])
        single_pyramid();
    
    // 第二个pyramid（绕X轴旋转180度）
    rotate([0,0,180])
    translate([0, 10, 0])
        single_pyramid();
}

// 显示模型
double_pyramid();

/* 参数说明：
 * height: 总高度（100mm）
 * top_radius: 顶面外切圆半径（15mm）
 * 
 * 几何特征：
 * - 每个pyramid有一个垂直于顶面的侧面
 * - 两个pyramid关于垂直面对称
 * - 每个pyramid由4个三角形侧面和1个顶面组成
 */
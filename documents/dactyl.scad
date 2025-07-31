// 基础参数
height = 10;           // 总高度
top_radius = 10;       // 顶面外切圆半径
$fn = 100;             // 平滑度

// 创建角锥体
module pyramid_prism() {
    // 定义顶点
    points = [
        // 底点（原点）
        [0, 0, height],         // 0: 底面点
        
        // 顶面的四个顶点
        [-top_radius, -top_radius, 0], // 1: 左后
        [top_radius, -top_radius, 0],  // 2: 右后
        [top_radius, top_radius, 0],   // 3: 右前
        [-top_radius, top_radius, 0]   // 4: 左前
    ];
    
    // 定义面
    faces = [
        [4, 3, 2, 1],    // 顶面（正方形）
        [0, 2, 1],       // 后侧面（三角形）
        [0, 2, 3],       // 右侧面（三角形）
        [0, 3, 4],       // 前侧面（三角形）
        [0, 4, 1]        // 左侧面（三角形）
    ];
    
    // 创建多面体
    polyhedron(
        points = points,
        faces = faces,
        convexity = 10
    );
}

// 显示模型
pyramid_prism();

/* 参数说明：
 * height: 总高度（100mm）
 * top_radius: 顶面外切圆半径（15mm）
 * 
 * 几何特征：
 * - 底面是一个点，位于原点(0,0,0)
 * - 顶面是一个正方形，边长为30mm（外切圆半径15mm）
 * - 4个三角形侧面
 * - 1个正方形顶面
 */
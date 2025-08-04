height = 10;           // total height
top_radius = 15;       
$fn = 100;             // smoothness

// create single pyramid
module single_pyramid() {
    // define vertex
    points = [
        // Origin
        [0, 0, height],   
        
        // Four vertices of the top face (adjusted so one side is perpendicular to the top face)
        [-top_radius, -top_radius, 0], // 1: left rear
        [top_radius, -top_radius, 0],  // 2: right rear
        [top_radius, 0, 0],           // 3: right front（y=0 the side is perpendicular）
        [-top_radius, 0, 0]           // 4: left front
    ];
    
    // define faces
    faces = [
        [4, 3, 2, 1],    // top face (square)
        [0, 1, 2],       // back face (triangle)
        [0, 2, 3],       // right face (triangle)
        [0, 3, 4],       // front face (triangle)
        [0, 4, 1]        // left face (triangle)
    ];
    
    // Create a polyhedron
    polyhedron(
        points = points,
        faces = faces,
        convexity = 10
    );
}

// Create a complete double pyramid
module double_pyramid() {
    // firsr pyramid
    translate([0, 10, 0])
        single_pyramid();
    
    // second pyramid
    rotate([0,0,180])
    translate([0, 10, 0])
        single_pyramid();
}

// show the model
double_pyramid();

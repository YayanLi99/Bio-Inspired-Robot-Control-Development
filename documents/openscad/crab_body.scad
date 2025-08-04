// Size parameters
r_tail = 10;        // tail radius
r_head = 7;         // head radius
z_tail = 12;        // thickness
z_head = 6;         
total_length = 27;  
body_smoothness = 5;  

// smoothness factor
smooth_factor = 1;   

module base_body() {
    hull() {
        // tail
        translate([-r_tail, 0, 0])
            scale([1, 1, z_tail/(2*r_tail)])
                sphere(r = r_tail, $fn=body_smoothness);
        
        // head - make sure the total length=7
        translate([0, 0, 0])
            scale([1, 1, z_head/(2*r_head)])
                sphere(r = r_head, $fn=body_smoothness);
    }
}

// merge the final smooth model
minkowski() {
    base_body();
    sphere(r = smooth_factor, $fn=50);
}

/* Parameters：
 * tail radius = 10
 * head radius = 7
 * total length = 27
 * nail thickness = 10
 * head thickness = 6
 * use minkowski() to smooth the model
 * hull() function smooth the link of tail and head
 */
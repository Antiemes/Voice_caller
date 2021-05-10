
$fn = 96;

module column(x, y, h){
    
    translate([x, y, 0]){
    
        difference(){
            cylinder(d=6, h=h);
            cylinder(d=3.2, h=h+1);
        }
    }
    
}

module screw_head(x, y, z, h){
    
    translate([x, y, z]){
        cylinder(d1=h, d2=0, h=h/2);
    }
    
}

CLEARANCE = 0.2;
PCB_LEN = 52.4  + CLEARANCE*2;
PCB_W  = 31.9*2 + CLEARANCE*2;
WALL_HEIGHT = 4+8;

COLUMN_HEIGHT = 4;
PAD = 2;

difference(){

union(){
translate([0, -PCB_W/2, 0]){
    difference(){
        
        union(){

            translate([-0.75, -0.75, -2]){
                cube([PCB_LEN+1.5, PCB_W+1.5, WALL_HEIGHT + 2 + 1.7]);
            }
            
            translate([-1.5, -1.5, -2]){
                cube([PCB_LEN+3, PCB_W+3, WALL_HEIGHT + 2]);
            }
        }

        translate([0, 0, 0]){
            cube([PCB_LEN, PCB_W, 30]);
        }
        
        //RJ45 1.
        #translate([-2, 20-15.7/2+PCB_W/2, COLUMN_HEIGHT + 1.6]){
            cube([3, 15.7, 12.8]);
        }
        
        //RJ45 2.
        #translate([-2, -20-15.7/2+PCB_W/2, COLUMN_HEIGHT + 1.6]){
            cube([3, 15.7, 12.8]);
        }
    }
}


column(4.7625 + CLEARANCE, 0, COLUMN_HEIGHT);
column(45.4025 + CLEARANCE, -27.94, COLUMN_HEIGHT);
column(45.4025 + CLEARANCE, 27.94, COLUMN_HEIGHT);
}

#screw_head(4.7625 + CLEARANCE, 0, -2, 7);
#screw_head(45.4025 + CLEARANCE, -27.94, -2, 7);
#screw_head(45.4025 + CLEARANCE, 27.94, -2, 7);


}
$fn = 90;

module column(x, y, h)
{	 
	translate([x, y, 0])
	{
		difference()
		{
			cylinder(d=6, h=h);
			cylinder(d=2.8, h=h+1);
		}
	}
}

module led_hole(x, y, z)
{	 
	translate([x, y, z])
	{
		cylinder(d=3.1, h=10);
	}
}

module btnSlit(x, y)
{	 
	translate([x, y, -2])
	difference()
	{	 
		translate([-4, -4, 0])
			cube([8, 8, 3]);
	  
		translate([-7.3/2, -7.3/2, 0])
			cube([7.3, 7.3, 3]);		
	}
}

module btn(x, y)
{	 
	translate([x, y, 0])
	{			 
		cylinder(d=4, h=13.5);
		
		difference()
		{
			translate([-10, -1.5, 0])
				cube([9, 3, 1 + 1.3]);
			
			translate([-8, -1.5, 0])
				cube([6, 3, 1]);
		}
	}
	
}

CLEARANCE = 0.2;
PCB_LEN = 52.4	+ CLEARANCE*2;
PCB_W  = 31.9*2 + CLEARANCE*2;
WALL_HEIGHT = 6.3+6;

COLUMN_HEIGHT = 18.7;
PAD = 2;

difference()
{
union()
{
translate([0, -PCB_W/2, 0])
{
	difference()
	{	 
		union()
		{	 
			translate([-1.5, -1.5, -2])
			{
				cube([PCB_LEN+3, PCB_W+3, WALL_HEIGHT+2]);
			}			 
		}

		translate([0, 0, 0])
		{
			cube([PCB_LEN, PCB_W, 30]);
		}
		
		translate([-0.9, -0.9, WALL_HEIGHT-2])
		{
			cube([PCB_LEN+1.5, PCB_W+1.5, 24]);
		}
		
		#led_hole(33,  -19.95 +PCB_W/2, -5);
		#led_hole(33,  0 +PCB_W/2, -5);
		
		//RJ45 1.
		#translate([-2, 20-15.7/2+PCB_W/2, 6])
		{
			cube([3, 15.7, 12.7]);
		}
		
		//RJ45 2.
		#translate([-2, -20-15.7/2+PCB_W/2, 6])
		{
			cube([3, 15.7, 12.7]);
		}		 

		//USB
		#translate([-2, -7.7/2+PCB_W/2, 7.7-2.8])
		{
			cube([3, 7.7, 2.8]);
		} 
		
		//speaker holes
		#translate([4.7625 + 14, PCB_W/2 +15.5, -2])
		rotate([0, 0, 45])
		for(i=[-12:3:12])
		{	 
			translate([i-1.5/2, abs(i) - 13, 0])
				cube([1.5, (13-abs(i))*2, 3]);
		}
		
		for(i=[-19.95:9.975:19.95])
		{
			btnSlit(45.4025 + CLEARANCE - 1.3, i + PCB_W/2);	
		}
		
	}
}


column(45.4025 + CLEARANCE, -27.94, COLUMN_HEIGHT);
column(45.4025 + CLEARANCE, 27.94, COLUMN_HEIGHT);

//buttons
for(i=[-19.95:9.975:19.95])
{	 
	btn(45.4025 + CLEARANCE, i);
}

// speaker
translate([4.7625 + 14, 0+15.5, 0])
difference()
{	 
	cylinder(d=30+CLEARANCE*2, h=2);
	cylinder(d=28+CLEARANCE*2, h=4); 
}

}

}

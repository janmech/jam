

inlets=1;
outlets=1;

mgraphics.init();
mgraphics.relative_coords = 0;
mgraphics.autofill = 0;
var alpha = 1.0;
var edge = [0,0];
var name = jsarguments[1];
var dict;
var args;
var value=[];
var opt=[];
var executed = false;
		
this.box.message("border", 0);
this.box.message("ignoreclick", 1);
this.box.message("sendtoback");

function init()
{
	dict = max.getrefdict(name);
 	args = dict.get("objargs");
	if(args)
	{
		if(args.getkeys() instanceof Array)
		{
			for(var i=0;i<args.getkeys().length;i++)
			{	
 				if(args.get(args.getkeys()[i]).get("optional")==1) 
				{
					opt=" (optional)";
				}
			value[i] = (args.get(args.getkeys()[i]).get("fullname") + " " + "(" + args.get(args.getkeys()[i]).get("type") + ")" + opt);
			}
		}
	else 
		{
			if(args.get(args.getkeys()).get("optional")==1) 
			{
				opt=" (optional)";
			}
			value = (args.get(args.getkeys()).get("fullname") + " " + "(" + args.get(args.getkeys()).get("type") + ")" + opt);
		}
	}
	dict.freepeer();
	if(args) { args.freepeer(); }	
}

init();


function clip(v, lo, hi)
{
	return (v < lo) ? lo : (v > hi) ? hi : v;
}

function adjustcolor(color, amount)
{
	var newcolor = [];

	if (color[0] + color [1] + color[2] < 0.6) {
		// always force lighten
		amount = Math.abs(amount);
	} else if (color[0] + color [1] + color[2] > 2.4) {
		// always force darken
		amount = - Math.abs(amount);
	}

	newcolor[0] = clip(color[0] + amount, 0., 1.);
	newcolor[1] = clip(color[1] + amount, 0., 1.);
	newcolor[2] = clip(color[2] + amount, 0., 1.);
	newcolor[3] = color[3];

	return newcolor;	
}

function paint()
{
	if(jsarguments[2]==null){jsarguments[2]=2};
	var offset = jsarguments[2];
	if(name){
    	with(mgraphics) 
    	{
			// set our font first, used in argEdgeCalc()
			select_font_face("Lato");
    		set_font_size(13);

			// calculate our maximum rectangle
        	argEdgeCalc(name); 
			
			// draw our rectangle and arrow
			var panelcolor = this.patcher.getattr("accentcolor");
			set_source_rgba(panelcolor);
			rectangle_rounded(20, 0, edge[0]+10, edge[1]+23, 8, 8, 8);
			fill();
			move_to(2, offset + 8);
			line_to(10, offset + 1);
			line_to(10, offset + 15); 
			close_path();
			fill();
			move_to(9, offset + 8);
			set_line_width(3);
			line_to(20, offset + 8);
			stroke();
    		move_to(25, 15);

			// draw our text
    		var textcolor = adjustcolor(panelcolor, -0.4);
			set_source_rgba(textcolor);
       		show_text("Arguments:");
			var inverttextcolor = adjustcolor(panelcolor, 0.4);
			inverttextcolor.push(1);	
			set_source_rgba(inverttextcolor);
			move_to(25,30);
			argDraw(name);
        	fill();
    	}
	}
}

function argDraw(name)
{
	if (value) {
		if(value instanceof Array)
		{
			for(var i=0;i<value.length;i++)
			{		
				mgraphics.show_text(value[i]);
				mgraphics.move_to(25,45+i*15);
			}
 		}
		else
		{
			mgraphics.show_text(value);
			mgraphics.move_to(25,60);
		}
	}
	return;
}	


function argEdgeCalc(name)
{
	if(value instanceof Array)
	{
		edge[0] = 68;
		for(var i=0;i<value.length;i++)
		{		
			var measure = mgraphics.text_measure(value[i]);
			edge[0]=Math.max(edge[0],measure[0]);
			edge[1]=value.length *15;
		}
		thebox();
	}
	else
	{
		edge[0]=mgraphics.text_measure(value)[0];
		edge[0]=Math.max(edge[0],68);
		edge[1]=15;
		thebox();
	}
	;
	return;
}	

function thebox() 
{
		if(!executed) {
			executed = true;
			this.box.message("rect", this.box.rect[0], this.box.rect[1], edge[0]+32,edge[1]+24);
			}
}

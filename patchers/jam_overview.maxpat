{
	"patcher" : 	{
		"fileversion" : 1,
		"appversion" : 		{
			"major" : 9,
			"minor" : 0,
			"revision" : 5,
			"architecture" : "x64",
			"modernui" : 1
		}
,
		"classnamespace" : "box",
		"openrect" : [ 549.0, 163.0, 666.85546875, 473.0 ],
		"openinpresentation" : 1,
		"gridsize" : [ 15.0, 15.0 ],
		"lefttoolbarpinned" : 2,
		"toptoolbarpinned" : 2,
		"righttoolbarpinned" : 2,
		"bottomtoolbarpinned" : 2,
		"toolbars_unpinned_last_save" : 15,
		"devicewidth" : 666.85546875,
		"title" : "jam Overview",
		"boxes" : [ 			{
				"box" : 				{
					"angle" : 270.0,
					"border" : 1,
					"bordercolor" : [ 1.0, 1.0, 1.0, 1.0 ],
					"grad1" : [ 0.172137149796092, 0.172137100044002, 0.172137113045018, 0.0 ],
					"grad2" : [ 0.172137149796092, 0.172137100044002, 0.172137113045018, 0.0 ],
					"id" : "obj-14",
					"maxclass" : "panel",
					"mode" : 1,
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 488.0, 361.0, 128.0, 128.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 223.0, 369.0, 229.0, 40.0 ],
					"proportion" : 0.5,
					"rounded" : 0
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-12",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 2,
					"outlettype" : [ "dictionary", "" ],
					"patching_rect" : [ 303.0, 274.0, 113.0, 22.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 303.0, 285.0, 113.0, 22.0 ],
					"text" : "jam.jit.gl.ilda.sketch"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-11",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 218.0, 274.0, 72.0, 22.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 218.0, 285.0, 72.0, 22.0 ],
					"text" : "jam.ilda.dict"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-10",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 5,
					"outlettype" : [ "", "", "int", "int", "list" ],
					"patching_rect" : [ 105.0, 274.0, 102.0, 22.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 102.0, 285.0, 102.0, 22.0 ],
					"text" : "jam.ilda.compose"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-9",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 2,
					"outlettype" : [ "", "list" ],
					"patching_rect" : [ 24.0, 274.0, 69.0, 22.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 19.0, 285.0, 69.0, 22.0 ],
					"text" : "jam.ilda.file"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-8",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 3,
					"outlettype" : [ "message", "int", "" ],
					"patching_rect" : [ 24.0, 196.0, 63.0, 22.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 19.0, 212.0, 63.0, 22.0 ],
					"text" : "jam.helios"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-7",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 4,
					"outlettype" : [ "list", "int", "", "anything" ],
					"patching_rect" : [ 159.0, 105.0, 98.0, 22.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 135.0, 143.0, 98.0, 22.0 ],
					"text" : "jam.dmxusbpro~"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-5",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 4,
					"outlettype" : [ "list", "int", "", "" ],
					"patching_rect" : [ 24.0, 105.0, 91.0, 22.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 19.0, 143.0, 91.0, 22.0 ],
					"text" : "jam.dmxusbpro"
				}

			}
, 			{
				"box" : 				{
					"bgcolor" : [ 0.125490196078431, 0.125490196078431, 0.125490196078431, 0.0 ],
					"fontname" : "Lato Light",
					"fontsize" : 24.0,
					"id" : "obj-2",
					"maxclass" : "textbutton",
					"numinlets" : 1,
					"numoutlets" : 3,
					"outlettype" : [ "", "", "int" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 145.0, 561.0, 240.0, 45.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 214.0, 364.0, 240.0, 45.0 ],
					"text" : "jam Documentation"
				}

			}
, 			{
				"box" : 				{
					"fontname" : "Lato Light",
					"fontsize" : 18.0,
					"id" : "obj-3",
					"maxclass" : "comment",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 41.0, 476.0, 167.0, 28.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 249.0, 329.0, 167.0, 28.0 ],
					"text" : "Click to get started"
				}

			}
, 			{
				"box" : 				{
					"hidden" : 1,
					"id" : "obj-4",
					"linecount" : 2,
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 15.0, 521.0, 219.0, 35.0 ],
					"text" : ";\rmax opendoc 01_jam_topic.maxvig.xml"
				}

			}
, 			{
				"box" : 				{
					"border" : 0,
					"fontface" : 0,
					"fontname" : "Lato Light",
					"fontsize" : 13.0,
					"id" : "obj-29",
					"linkbold" : 1,
					"linkcolor" : [ 0.85, 0.85, 0.85, 1.0 ],
					"maxclass" : "markup",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 19.0, 381.0, 262.0, 39.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 15.0, 443.0, 305.0, 22.0 ],
					"saved_attribute_attributes" : 					{
						"linkcolor" : 						{
							"expression" : "themecolor.live_control_fg"
						}
,
						"textcolor" : 						{
							"expression" : "themecolor.live_control_fg"
						}

					}
,
					"text" : "Project on GitHub <a href=\"https://github.com/janmech/jam\">https://github.com/janmech/jam</a>",
					"textcolor" : [ 0.85, 0.85, 0.85, 1.0 ]
				}

			}
, 			{
				"box" : 				{
					"border" : 0,
					"fontface" : 0,
					"fontname" : "Lato Light",
					"fontsize" : 13.0,
					"id" : "obj-25",
					"linkbold" : 1,
					"linkcolor" : [ 0.85, 0.85, 0.85, 1.0 ],
					"maxclass" : "markup",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 395.0, 442.0, 267.0, 34.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 348.0, 443.0, 290.0, 22.0 ],
					"saved_attribute_attributes" : 					{
						"linkcolor" : 						{
							"expression" : "themecolor.live_control_fg"
						}
,
						"textcolor" : 						{
							"expression" : "themecolor.live_control_fg"
						}

					}
,
					"text" : "created by <a href=\"https://janmech.net\">Jan Mech</a> © 2025 The MIT Lisense",
					"textcolor" : [ 0.85, 0.85, 0.85, 1.0 ]
				}

			}
, 			{
				"box" : 				{
					"border" : 0,
					"fontface" : 0,
					"fontname" : "Lato Light",
					"fontsize" : 16.0,
					"id" : "obj-23",
					"linkbold" : 1,
					"linkcolor" : [ 0.85, 0.85, 0.85, 1.0 ],
					"maxclass" : "markup",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 814.0, 119.0, 185.0, 50.0 ],
					"saved_attribute_attributes" : 					{
						"linkcolor" : 						{
							"expression" : "themecolor.live_control_fg"
						}
,
						"textcolor" : 						{
							"expression" : "themecolor.live_control_fg"
						}

					}
,
					"text" : "\n<a href=\"jam.ilda.compose\">jam.ilda.compose</a> - Create and mofify ILDA files for laser animation<br/>\n<a href=\"jam.ilda.dict\">jam.ilda.dict</a> - Create a dictionary from an ILDA file with file information<br/>\n<a href=\"jam.ilda.file\">jam.ilda.file</a> - Load an ILDA file (laser animation file) from disk<br/>\n<a href=\"jam.jit.gl.ilda.sketch\">jam.jit.gl.ilda.sketch</a> - Render frames from an ILDA file to an Open GL context",
					"textcolor" : [ 0.85, 0.85, 0.85, 1.0 ]
				}

			}
, 			{
				"box" : 				{
					"fontname" : "Lato Light",
					"fontsize" : 14.0,
					"id" : "obj-24",
					"maxclass" : "comment",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 24.0, 232.0, 174.0, 23.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 15.0, 253.0, 174.0, 23.0 ],
					"text" : "ILDA File Objects"
				}

			}
, 			{
				"box" : 				{
					"border" : 0,
					"fontface" : 0,
					"fontname" : "Lato Light",
					"fontsize" : 16.0,
					"id" : "obj-22",
					"linkbold" : 1,
					"linkcolor" : [ 0.85, 0.85, 0.85, 1.0 ],
					"maxclass" : "markup",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 814.0, 95.0, 146.0, 17.0 ],
					"saved_attribute_attributes" : 					{
						"linkcolor" : 						{
							"expression" : "themecolor.live_control_fg"
						}
,
						"textcolor" : 						{
							"expression" : "themecolor.live_control_fg"
						}

					}
,
					"text" : "<a href=\"jam.helios\">jam.helios</a> - Connect to a Helios ILDA DAC",
					"textcolor" : [ 0.85, 0.85, 0.85, 1.0 ]
				}

			}
, 			{
				"box" : 				{
					"fontname" : "Lato Light",
					"fontsize" : 14.0,
					"id" : "obj-21",
					"maxclass" : "comment",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 24.0, 164.0, 174.0, 23.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 15.0, 180.0, 174.0, 23.0 ],
					"text" : "ILDA Interface Objects"
				}

			}
, 			{
				"box" : 				{
					"autofit" : 1,
					"forceaspect" : 1,
					"id" : "obj-19",
					"maxclass" : "fpic",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "jit_matrix" ],
					"patching_rect" : [ 631.0, 325.0, 100.0, 100.0 ],
					"pic" : "Macintosh HD:/Users/janmech/Documents/Workspace/Xcode/jam/icon.png",
					"presentation" : 1,
					"presentation_rect" : [ 15.0, 26.0, 58.0, 58.0 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-18",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 824.0, 205.0, 79.0, 22.0 ],
					"text" : "prepend help"
				}

			}
, 			{
				"box" : 				{
					"border" : 0,
					"fontface" : 0,
					"fontname" : "Lato Light",
					"fontsize" : 16.0,
					"id" : "obj-17",
					"linkbold" : 1,
					"linkcolor" : [ 0.85, 0.85, 0.85, 1.0 ],
					"maxclass" : "markup",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 814.0, 47.0, 226.0, 41.0 ],
					"saved_attribute_attributes" : 					{
						"linkcolor" : 						{
							"expression" : "themecolor.live_control_fg"
						}
,
						"textcolor" : 						{
							"expression" : "themecolor.live_control_fg"
						}

					}
,
					"text" : "<a href=\"jam.dmxusbpro\">jam.dmxusbpro</a> -Connect to the ENTTEC DMX USB Pro interface. Conrol DMX data with lists<br/>\n<a href=\"jam.dmxusbpro~\">jam.dmxusbpro~</a> - Connect to the ENTTEC DMX USB Pro interface. Conrol DMX data with signals<br/>",
					"textcolor" : [ 0.85, 0.85, 0.85, 1.0 ]
				}

			}
, 			{
				"box" : 				{
					"fontname" : "Lato Light",
					"fontsize" : 14.0,
					"id" : "obj-13",
					"maxclass" : "comment",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 24.0, 65.0, 174.0, 23.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 15.0, 114.0, 174.0, 23.0 ],
					"text" : "DMX Interface Objects"
				}

			}
, 			{
				"box" : 				{
					"border" : 0,
					"fontface" : 0,
					"fontname" : "Lato Light",
					"fontsize" : 16.0,
					"id" : "obj-1",
					"linkcolor" : [ 0.85, 0.85, 0.85, 1.0 ],
					"linkunderline" : 0,
					"maxclass" : "markup",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 27.0, 12.0, 712.0, 47.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 82.0, 26.0, 556.0, 73.0 ],
					"saved_attribute_attributes" : 					{
						"linkcolor" : 						{
							"expression" : "themecolor.live_control_fg"
						}
,
						"textcolor" : 						{
							"expression" : "themecolor.live_control_fg"
						}

					}
,
					"text" : "The jam package is a collections of externals for working with DMX devices via an ENTTEC DMX USB Pro DMX interface and with show laser projectors that have an ILDA interface via an Helios ILDA DAC.",
					"textcolor" : [ 0.85, 0.85, 0.85, 1.0 ]
				}

			}
, 			{
				"box" : 				{
					"hidden" : 1,
					"id" : "obj-6",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 824.0, 239.0, 53.0, 22.0 ],
					"text" : "pcontrol"
				}

			}
, 			{
				"box" : 				{
					"fontname" : "Arial",
					"fontsize" : 13.0,
					"hidden" : 1,
					"id" : "obj-26",
					"linecount" : 2,
					"maxclass" : "message",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 395.0, 486.0, 139.0, 38.0 ],
					"text" : ";\rmax launchbrowser $1"
				}

			}
, 			{
				"box" : 				{
					"fontname" : "Lato Light",
					"fontsize" : 13.0,
					"id" : "obj-20",
					"maxclass" : "comment",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 399.0, 398.0, 219.0, 22.0 ],
					"presentation" : 1,
					"presentation_rect" : [ 613.0, 440.0, 37.0, 22.0 ],
					"text" : "v.2.0",
					"textcolor" : [ 0.426676, 0.426663, 0.42667, 1.0 ]
				}

			}
 ],
		"lines" : [ 			{
				"patchline" : 				{
					"destination" : [ "obj-18", 0 ],
					"source" : [ "obj-17", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-6", 0 ],
					"source" : [ "obj-18", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-4", 0 ],
					"hidden" : 1,
					"source" : [ "obj-2", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-18", 0 ],
					"source" : [ "obj-22", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-18", 0 ],
					"source" : [ "obj-23", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-26", 0 ],
					"source" : [ "obj-25", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-26", 0 ],
					"source" : [ "obj-29", 0 ]
				}

			}
 ],
		"originid" : "pat-42",
		"dependency_cache" : [ 			{
				"name" : "icon.png",
				"bootpath" : "~/Documents/Workspace/Xcode/jam",
				"patcherrelativepath" : "..",
				"type" : "PNG",
				"implicit" : 1
			}
, 			{
				"name" : "jam.dmxusbpro.mxo",
				"type" : "iLaX"
			}
, 			{
				"name" : "jam.dmxusbpro~.mxo",
				"type" : "iLaX"
			}
, 			{
				"name" : "jam.helios.mxo",
				"type" : "iLaX"
			}
, 			{
				"name" : "jam.ilda.compose.mxo",
				"type" : "iLaX"
			}
, 			{
				"name" : "jam.ilda.dict.mxo",
				"type" : "iLaX"
			}
, 			{
				"name" : "jam.ilda.file.mxo",
				"type" : "iLaX"
			}
, 			{
				"name" : "jam.jit.gl.ilda.sketch.mxo",
				"type" : "iLaX"
			}
 ],
		"autosave" : 0,
		"styles" : [ 			{
				"name" : "AudioStatus_Menu",
				"default" : 				{
					"bgfillcolor" : 					{
						"angle" : 270.0,
						"autogradient" : 0,
						"color" : [ 0.294118, 0.313726, 0.337255, 1 ],
						"color1" : [ 0.454902, 0.462745, 0.482353, 0.0 ],
						"color2" : [ 0.290196, 0.309804, 0.301961, 1.0 ],
						"proportion" : 0.39,
						"type" : "color"
					}

				}
,
				"parentstyle" : "",
				"multi" : 0
			}
, 			{
				"name" : "ksliderWhite",
				"default" : 				{
					"color" : [ 1.0, 1.0, 1.0, 1.0 ]
				}
,
				"parentstyle" : "",
				"multi" : 0
			}
, 			{
				"name" : "newobjBlue-1",
				"default" : 				{
					"accentcolor" : [ 0.317647, 0.654902, 0.976471, 1.0 ]
				}
,
				"parentstyle" : "",
				"multi" : 0
			}
, 			{
				"name" : "newobjBrown-1",
				"default" : 				{
					"accentcolor" : [ 0.654902, 0.572549, 0.376471, 1.0 ]
				}
,
				"parentstyle" : "",
				"multi" : 0
			}
, 			{
				"name" : "newobjCyan-1",
				"default" : 				{
					"accentcolor" : [ 0.029546, 0.773327, 0.821113, 1.0 ]
				}
,
				"parentstyle" : "",
				"multi" : 0
			}
, 			{
				"name" : "newobjGreen-1",
				"default" : 				{
					"accentcolor" : [ 0.0, 0.533333, 0.168627, 1.0 ]
				}
,
				"parentstyle" : "",
				"multi" : 0
			}
, 			{
				"name" : "newobjYellow-1",
				"default" : 				{
					"accentcolor" : [ 0.82517, 0.78181, 0.059545, 1.0 ],
					"fontsize" : [ 12.059008 ]
				}
,
				"parentstyle" : "",
				"multi" : 0
			}
, 			{
				"name" : "numberGold-1",
				"default" : 				{
					"accentcolor" : [ 0.764706, 0.592157, 0.101961, 1.0 ]
				}
,
				"parentstyle" : "",
				"multi" : 0
			}
, 			{
				"name" : "rsliderGold",
				"default" : 				{
					"bgcolor" : [ 0.764706, 0.592157, 0.101961, 1.0 ],
					"color" : [ 0.646639, 0.821777, 0.854593, 1.0 ]
				}
,
				"parentstyle" : "",
				"multi" : 0
			}
 ]
	}

}

{
	"patcher" : 	{
		"fileversion" : 1,
		"appversion" : 		{
			"major" : 9,
			"minor" : 0,
			"revision" : 6,
			"architecture" : "x64",
			"modernui" : 1
		}
,
		"classnamespace" : "box",
		"rect" : [ 84.0, 100.0, 887.0, 628.0 ],
		"gridsize" : [ 15.0, 15.0 ],
		"showrootpatcherontab" : 0,
		"showontab" : 0,
		"boxes" : [ 			{
				"box" : 				{
					"id" : "obj-7",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 0,
					"patcher" : 					{
						"fileversion" : 1,
						"appversion" : 						{
							"major" : 9,
							"minor" : 0,
							"revision" : 6,
							"architecture" : "x64",
							"modernui" : 1
						}
,
						"classnamespace" : "box",
						"rect" : [ 0.0, 26.0, 887.0, 602.0 ],
						"gridsize" : [ 15.0, 15.0 ],
						"showontab" : 1,
						"boxes" : [ 							{
								"box" : 								{
									"hidden" : 1,
									"id" : "obj-7",
									"linecount" : 2,
									"maxclass" : "message",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 391.0, 558.0, 97.0, 35.0 ],
									"text" : ";\rmax opendoc $1"
								}

							}
, 							{
								"box" : 								{
									"border" : 0,
									"fontface" : 0,
									"fontname" : "Lato Light",
									"fontsize" : 13.0,
									"id" : "obj-33",
									"linkbold" : 1,
									"linkcolor" : [ 0.85, 0.85, 0.85, 1.0 ],
									"maxclass" : "markup",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 391.0, 376.0, 360.0, 168.0 ],
									"saved_attribute_attributes" : 									{
										"linkcolor" : 										{
											"expression" : "themecolor.live_control_fg"
										}
,
										"textcolor" : 										{
											"expression" : "themecolor.live_control_fg_off"
										}

									}
,
									"text" : "<b>Frame Format Handling</b><br/><br/>\n<b>jam.ilda.compose</b> always uses the FORMAT 5  (2D Coordinates with True Color). Imported 3D frane will be flattened to 2D, and indexed colors will be converted to true colors (RGB colors) using the ILDA standard pallet. <br/><br/>\n\nFor mor information about <b>ILDA file formats</b> and color handling, please refer to the <a href=\"02_jam_ilda_topic.maxvig.xml\">ILDA Files and ILDA Interface</a> topic in the documentation browser.\n",
									"textcolor" : [ 0.85, 0.85, 0.85, 1.0 ]
								}

							}
, 							{
								"box" : 								{
									"fontname" : "Lato Light",
									"id" : "obj-2",
									"linecount" : 2,
									"maxclass" : "comment",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 9.0, 109.0, 680.0, 35.0 ],
									"text" : "You can import any ILDA file loaded with jam.ilda.file by sending the file reference. Frames are copied to jam.ilda.compose and inserted after the last frame."
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-11",
									"maxclass" : "dict.view",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 31.0, 383.0, 332.0, 154.0 ]
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-6",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 31.0, 349.0, 72.0, 22.0 ],
									"text" : "jam.ilda.dict"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-3",
									"maxclass" : "message",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 124.0, 248.0, 42.0, 22.0 ],
									"text" : "import"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-4",
									"maxclass" : "message",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 31.0, 248.0, 88.0, 22.0 ],
									"text" : "import fancy.ild"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-1",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 2,
									"outlettype" : [ "", "list" ],
									"patching_rect" : [ 31.0, 279.0, 69.0, 22.0 ],
									"text" : "jam.ilda.file"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-5",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 5,
									"outlettype" : [ "", "", "int", "int", "list" ],
									"patching_rect" : [ 31.0, 319.0, 102.0, 22.0 ],
									"text" : "jam.ilda.compose"
								}

							}
, 							{
								"box" : 								{
									"fontname" : "Lato Light",
									"fontsize" : 24.0,
									"id" : "obj-18",
									"maxclass" : "comment",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 10.0, 69.599853515625, 254.0, 35.0 ],
									"text" : "ILDA file import"
								}

							}
, 							{
								"box" : 								{
									"border" : 0,
									"filename" : "helpname.js",
									"id" : "obj-12",
									"ignoreclick" : 1,
									"jsarguments" : [ "jam.ilda.compose" ],
									"maxclass" : "jsui",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"parameter_enable" : 0,
									"patching_rect" : [ 10.0, 10.0, 365.527984619140625, 57.599853515625 ]
								}

							}
 ],
						"lines" : [ 							{
								"patchline" : 								{
									"destination" : [ "obj-5", 0 ],
									"source" : [ "obj-1", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-1", 0 ],
									"source" : [ "obj-3", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-7", 0 ],
									"hidden" : 1,
									"source" : [ "obj-33", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-1", 0 ],
									"source" : [ "obj-4", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-6", 0 ],
									"source" : [ "obj-5", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-11", 0 ],
									"source" : [ "obj-6", 0 ]
								}

							}
 ]
					}
,
					"patching_rect" : [ 121.0, 178.0, 114.0, 22.0 ],
					"text" : "p \"ILDA files\" import"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-6",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 0,
					"patcher" : 					{
						"fileversion" : 1,
						"appversion" : 						{
							"major" : 9,
							"minor" : 0,
							"revision" : 6,
							"architecture" : "x64",
							"modernui" : 1
						}
,
						"classnamespace" : "box",
						"rect" : [ 0.0, 26.0, 887.0, 602.0 ],
						"gridsize" : [ 15.0, 15.0 ],
						"showontab" : 1,
						"boxes" : [ 							{
								"box" : 								{
									"border" : 0,
									"fontface" : 0,
									"fontname" : "Lato Light",
									"fontsize" : 13.0,
									"id" : "obj-33",
									"maxclass" : "markup",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 13.0, 107.0, 743.0, 90.0 ],
									"saved_attribute_attributes" : 									{
										"textcolor" : 										{
											"expression" : "themecolor.live_control_fg_off"
										}

									}
,
									"text" : "ILDA files contain a <i>Company Name</i> and a <i>Frame Name</i> in every frame header. <br/>\nThese are meta data with no functional value. The <i>Company Name</i> allows to name a creator of the file. You can set your own <i>Company Name</i> using the companyname attribute.\n<br/><br/>\n<b>jam.ilda.compose</b> uses the frame index as <i>Frame Name</i>. Using the frameprefix you and add a custom prefix to the frame name.\n",
									"textcolor" : [ 0.85, 0.85, 0.85, 1.0 ]
								}

							}
, 							{
								"box" : 								{
									"bubble" : 1,
									"fontname" : "Lato Light",
									"id" : "obj-15",
									"linecount" : 3,
									"maxclass" : "comment",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 548.0, 227.0, 163.0, 54.0 ],
									"text" : "Set your own Company Name and Frame Name prefix"
								}

							}
, 							{
								"box" : 								{
									"bubble" : 1,
									"fontname" : "Lato Light",
									"id" : "obj-14",
									"linecount" : 2,
									"maxclass" : "comment",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 186.0, 221.0, 93.0, 39.0 ],
									"text" : "create some frames"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-10",
									"maxclass" : "message",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 10.0, 280.0, 35.0, 22.0 ],
									"text" : "clear"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-8",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "appendframe" ],
									"patcher" : 									{
										"fileversion" : 1,
										"appversion" : 										{
											"major" : 9,
											"minor" : 0,
											"revision" : 6,
											"architecture" : "x64",
											"modernui" : 1
										}
,
										"classnamespace" : "box",
										"rect" : [ 189.0, 100.0, 988.0, 730.0 ],
										"gridsize" : [ 15.0, 15.0 ],
										"boxes" : [ 											{
												"box" : 												{
													"bubble" : 1,
													"fontname" : "Lato Light",
													"id" : "obj-12",
													"linecount" : 4,
													"maxclass" : "comment",
													"numinlets" : 1,
													"numoutlets" : 0,
													"patching_rect" : [ 276.0, 360.0, 137.0, 68.0 ],
													"text" : "...and finally add some gradually changing shapes to each frame!"
												}

											}
, 											{
												"box" : 												{
													"bubble" : 1,
													"fontname" : "Lato Light",
													"id" : "obj-10",
													"linecount" : 2,
													"maxclass" : "comment",
													"numinlets" : 1,
													"numoutlets" : 0,
													"patching_rect" : [ 455.0, 314.5, 137.0, 39.0 ],
													"text" : "...set the drawing color..."
												}

											}
, 											{
												"box" : 												{
													"bubble" : 1,
													"fontname" : "Lato Light",
													"id" : "obj-8",
													"linecount" : 2,
													"maxclass" : "comment",
													"numinlets" : 1,
													"numoutlets" : 0,
													"patching_rect" : [ 542.0, 236.0, 162.0, 39.0 ],
													"text" : "add an new frame for each number..."
												}

											}
, 											{
												"box" : 												{
													"bubble" : 1,
													"fontname" : "Lato Light",
													"id" : "obj-4",
													"linecount" : 2,
													"maxclass" : "comment",
													"numinlets" : 1,
													"numoutlets" : 0,
													"patching_rect" : [ 320.0, 124.5, 162.0, 39.0 ],
													"text" : "fold to count up to 1. and down agin to 0."
												}

											}
, 											{
												"box" : 												{
													"bubble" : 1,
													"fontname" : "Lato Light",
													"id" : "obj-3",
													"linecount" : 2,
													"maxclass" : "comment",
													"numinlets" : 1,
													"numoutlets" : 0,
													"patching_rect" : [ 236.0, 64.0, 162.0, 39.0 ],
													"text" : "generate number from 0. to 2. in 100 steps"
												}

											}
, 											{
												"box" : 												{
													"id" : "obj-112",
													"maxclass" : "newobj",
													"numinlets" : 2,
													"numoutlets" : 1,
													"outlettype" : [ "float" ],
													"patching_rect" : [ 28.0, 381.0, 36.0, 22.0 ],
													"text" : "+ 90."
												}

											}
, 											{
												"box" : 												{
													"id" : "obj-111",
													"maxclass" : "newobj",
													"numinlets" : 2,
													"numoutlets" : 1,
													"outlettype" : [ "float" ],
													"patching_rect" : [ 72.0, 351.0, 36.0, 22.0 ],
													"text" : "+ 60."
												}

											}
, 											{
												"box" : 												{
													"id" : "obj-108",
													"maxclass" : "newobj",
													"numinlets" : 2,
													"numoutlets" : 1,
													"outlettype" : [ "float" ],
													"patching_rect" : [ 121.0, 319.0, 36.0, 22.0 ],
													"text" : "+ 30."
												}

											}
, 											{
												"box" : 												{
													"id" : "obj-105",
													"maxclass" : "message",
													"numinlets" : 2,
													"numoutlets" : 1,
													"outlettype" : [ "" ],
													"patching_rect" : [ 28.0, 413.0, 123.0, 22.0 ],
													"text" : "circle 0. 0. 0.6 90. $1."
												}

											}
, 											{
												"box" : 												{
													"id" : "obj-104",
													"maxclass" : "message",
													"numinlets" : 2,
													"numoutlets" : 1,
													"outlettype" : [ "" ],
													"patching_rect" : [ 72.0, 383.0, 123.0, 22.0 ],
													"text" : "circle 0. 0. 0.7 60. $1."
												}

											}
, 											{
												"box" : 												{
													"id" : "obj-91",
													"maxclass" : "message",
													"numinlets" : 2,
													"numoutlets" : 1,
													"outlettype" : [ "" ],
													"patching_rect" : [ 121.0, 351.0, 123.0, 22.0 ],
													"text" : "circle 0. 0. 0.8 30. $1."
												}

											}
, 											{
												"box" : 												{
													"id" : "obj-18",
													"maxclass" : "newobj",
													"numinlets" : 2,
													"numoutlets" : 1,
													"outlettype" : [ "float" ],
													"patching_rect" : [ 375.0, 258.0, 29.5, 22.0 ],
													"text" : "!- 1."
												}

											}
, 											{
												"box" : 												{
													"id" : "obj-17",
													"maxclass" : "newobj",
													"numinlets" : 2,
													"numoutlets" : 1,
													"outlettype" : [ "float" ],
													"patching_rect" : [ 176.0, 99.0, 32.0, 22.0 ],
													"text" : "/ 50."
												}

											}
, 											{
												"box" : 												{
													"id" : "obj-11",
													"maxclass" : "newobj",
													"numinlets" : 2,
													"numoutlets" : 3,
													"outlettype" : [ "bang", "bang", "int" ],
													"patching_rect" : [ 138.0, 64.0, 57.0, 22.0 ],
													"text" : "uzi 101 0"
												}

											}
, 											{
												"box" : 												{
													"id" : "obj-13",
													"maxclass" : "newobj",
													"numinlets" : 1,
													"numoutlets" : 2,
													"outlettype" : [ "float", "float" ],
													"patching_rect" : [ 176.0, 223.0, 176.0, 22.0 ],
													"text" : "t f f"
												}

											}
, 											{
												"box" : 												{
													"id" : "obj-82",
													"maxclass" : "message",
													"numinlets" : 2,
													"numoutlets" : 1,
													"outlettype" : [ "" ],
													"patching_rect" : [ 176.0, 316.0, 113.0, 22.0 ],
													"text" : "circle 0. 0. 0.9 0. $1"
												}

											}
, 											{
												"box" : 												{
													"id" : "obj-9",
													"maxclass" : "newobj",
													"numinlets" : 2,
													"numoutlets" : 1,
													"outlettype" : [ "" ],
													"patching_rect" : [ 333.0, 289.0, 61.0, 22.0 ],
													"text" : "pack 0. 0."
												}

											}
, 											{
												"box" : 												{
													"id" : "obj-6",
													"maxclass" : "message",
													"numinlets" : 2,
													"numoutlets" : 1,
													"outlettype" : [ "" ],
													"patching_rect" : [ 333.0, 316.0, 107.0, 22.0 ],
													"text" : "drawcolor 1. $2 $1"
												}

											}
, 											{
												"box" : 												{
													"id" : "obj-87",
													"maxclass" : "newobj",
													"numinlets" : 2,
													"numoutlets" : 1,
													"outlettype" : [ "float" ],
													"patching_rect" : [ 176.0, 258.0, 40.0, 22.0 ],
													"text" : "* 360."
												}

											}
, 											{
												"box" : 												{
													"id" : "obj-98",
													"maxclass" : "newobj",
													"numinlets" : 1,
													"numoutlets" : 2,
													"outlettype" : [ "float", "appendframe" ],
													"patching_rect" : [ 176.0, 196.0, 354.0, 22.0 ],
													"text" : "t f appendframe"
												}

											}
, 											{
												"box" : 												{
													"id" : "obj-5",
													"maxclass" : "newobj",
													"numinlets" : 3,
													"numoutlets" : 1,
													"outlettype" : [ "" ],
													"patching_rect" : [ 176.0, 133.0, 130.0, 22.0 ],
													"text" : "pong 0. 1. @mode fold"
												}

											}
, 											{
												"box" : 												{
													"comment" : "",
													"id" : "obj-1",
													"index" : 1,
													"maxclass" : "inlet",
													"numinlets" : 0,
													"numoutlets" : 1,
													"outlettype" : [ "bang" ],
													"patching_rect" : [ 138.0, 19.0, 30.0, 30.0 ]
												}

											}
, 											{
												"box" : 												{
													"comment" : "",
													"id" : "obj-7",
													"index" : 1,
													"maxclass" : "outlet",
													"numinlets" : 1,
													"numoutlets" : 0,
													"patching_rect" : [ 250.0, 548.0, 30.0, 30.0 ]
												}

											}
 ],
										"lines" : [ 											{
												"patchline" : 												{
													"destination" : [ "obj-11", 0 ],
													"source" : [ "obj-1", 0 ]
												}

											}
, 											{
												"patchline" : 												{
													"destination" : [ "obj-7", 0 ],
													"midpoints" : [ 81.5, 531.0, 259.5, 531.0 ],
													"source" : [ "obj-104", 0 ]
												}

											}
, 											{
												"patchline" : 												{
													"destination" : [ "obj-7", 0 ],
													"midpoints" : [ 37.5, 531.02734375, 259.5, 531.02734375 ],
													"source" : [ "obj-105", 0 ]
												}

											}
, 											{
												"patchline" : 												{
													"destination" : [ "obj-91", 0 ],
													"source" : [ "obj-108", 0 ]
												}

											}
, 											{
												"patchline" : 												{
													"destination" : [ "obj-17", 0 ],
													"source" : [ "obj-11", 2 ]
												}

											}
, 											{
												"patchline" : 												{
													"destination" : [ "obj-104", 0 ],
													"source" : [ "obj-111", 0 ]
												}

											}
, 											{
												"patchline" : 												{
													"destination" : [ "obj-105", 0 ],
													"source" : [ "obj-112", 0 ]
												}

											}
, 											{
												"patchline" : 												{
													"destination" : [ "obj-18", 0 ],
													"order" : 0,
													"source" : [ "obj-13", 1 ]
												}

											}
, 											{
												"patchline" : 												{
													"destination" : [ "obj-87", 0 ],
													"source" : [ "obj-13", 0 ]
												}

											}
, 											{
												"patchline" : 												{
													"destination" : [ "obj-9", 0 ],
													"order" : 1,
													"source" : [ "obj-13", 1 ]
												}

											}
, 											{
												"patchline" : 												{
													"destination" : [ "obj-5", 0 ],
													"source" : [ "obj-17", 0 ]
												}

											}
, 											{
												"patchline" : 												{
													"destination" : [ "obj-9", 1 ],
													"source" : [ "obj-18", 0 ]
												}

											}
, 											{
												"patchline" : 												{
													"destination" : [ "obj-98", 0 ],
													"source" : [ "obj-5", 0 ]
												}

											}
, 											{
												"patchline" : 												{
													"destination" : [ "obj-7", 0 ],
													"midpoints" : [ 342.5, 532.06640625, 259.5, 532.06640625 ],
													"source" : [ "obj-6", 0 ]
												}

											}
, 											{
												"patchline" : 												{
													"destination" : [ "obj-7", 0 ],
													"midpoints" : [ 185.5, 529.97265625, 259.5, 529.97265625 ],
													"source" : [ "obj-82", 0 ]
												}

											}
, 											{
												"patchline" : 												{
													"destination" : [ "obj-108", 0 ],
													"order" : 1,
													"source" : [ "obj-87", 0 ]
												}

											}
, 											{
												"patchline" : 												{
													"destination" : [ "obj-111", 0 ],
													"order" : 2,
													"source" : [ "obj-87", 0 ]
												}

											}
, 											{
												"patchline" : 												{
													"destination" : [ "obj-112", 0 ],
													"order" : 3,
													"source" : [ "obj-87", 0 ]
												}

											}
, 											{
												"patchline" : 												{
													"destination" : [ "obj-82", 0 ],
													"order" : 0,
													"source" : [ "obj-87", 0 ]
												}

											}
, 											{
												"patchline" : 												{
													"destination" : [ "obj-6", 0 ],
													"source" : [ "obj-9", 0 ]
												}

											}
, 											{
												"patchline" : 												{
													"destination" : [ "obj-7", 0 ],
													"midpoints" : [ 130.5, 530.51171875, 259.5, 530.51171875 ],
													"source" : [ "obj-91", 0 ]
												}

											}
, 											{
												"patchline" : 												{
													"destination" : [ "obj-13", 0 ],
													"source" : [ "obj-98", 0 ]
												}

											}
, 											{
												"patchline" : 												{
													"destination" : [ "obj-7", 0 ],
													"midpoints" : [ 520.5, 531.74609375, 259.5, 531.74609375 ],
													"source" : [ "obj-98", 1 ]
												}

											}
 ]
									}
,
									"patching_rect" : [ 154.0, 266.0, 107.0, 22.0 ],
									"text" : "p make_animation"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-17",
									"maxclass" : "button",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "bang" ],
									"parameter_enable" : 0,
									"patching_rect" : [ 154.0, 227.0, 24.0, 24.0 ]
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-7",
									"maxclass" : "message",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 60.0, 243.0, 42.0, 22.0 ],
									"text" : "export"
								}

							}
, 							{
								"box" : 								{
									"attr" : "frameprefix",
									"id" : "obj-5",
									"maxclass" : "attrui",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"parameter_enable" : 0,
									"patching_rect" : [ 342.0, 259.0, 190.0, 22.0 ]
								}

							}
, 							{
								"box" : 								{
									"attr" : "companyname",
									"id" : "obj-4",
									"maxclass" : "attrui",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"parameter_enable" : 0,
									"patching_rect" : [ 342.0, 228.0, 190.0, 22.0 ]
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-3",
									"maxclass" : "dict.view",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 60.0, 398.0, 273.0, 128.0 ]
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-2",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 60.0, 351.0, 72.0, 22.0 ],
									"text" : "jam.ilda.dict"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-1",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 5,
									"outlettype" : [ "", "", "int", "int", "list" ],
									"patching_rect" : [ 60.0, 319.0, 102.0, 22.0 ],
									"text" : "jam.ilda.compose"
								}

							}
, 							{
								"box" : 								{
									"fontname" : "Lato Light",
									"fontsize" : 24.0,
									"id" : "obj-18",
									"maxclass" : "comment",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 10.0, 69.599853515625, 254.0, 35.0 ],
									"text" : "Meta Data"
								}

							}
, 							{
								"box" : 								{
									"border" : 0,
									"filename" : "helpname.js",
									"id" : "obj-12",
									"ignoreclick" : 1,
									"jsarguments" : [ "jam.ilda.compose" ],
									"maxclass" : "jsui",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"parameter_enable" : 0,
									"patching_rect" : [ 10.0, 10.0, 365.527984619140625, 57.599853515625 ]
								}

							}
 ],
						"lines" : [ 							{
								"patchline" : 								{
									"destination" : [ "obj-2", 0 ],
									"source" : [ "obj-1", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-1", 0 ],
									"source" : [ "obj-10", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-8", 0 ],
									"source" : [ "obj-17", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-3", 0 ],
									"source" : [ "obj-2", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-1", 0 ],
									"midpoints" : [ 351.5, 311.1484375, 69.5, 311.1484375 ],
									"source" : [ "obj-4", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-1", 0 ],
									"midpoints" : [ 351.5, 310.71484375, 69.5, 310.71484375 ],
									"source" : [ "obj-5", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-1", 0 ],
									"source" : [ "obj-7", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-1", 0 ],
									"midpoints" : [ 163.5, 312.234375, 69.5, 312.234375 ],
									"source" : [ "obj-8", 0 ]
								}

							}
 ]
					}
,
					"patching_rect" : [ 264.0, 259.0, 80.0, 22.0 ],
					"text" : "p \"meta data\""
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-5",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 0,
					"patcher" : 					{
						"fileversion" : 1,
						"appversion" : 						{
							"major" : 9,
							"minor" : 0,
							"revision" : 6,
							"architecture" : "x64",
							"modernui" : 1
						}
,
						"classnamespace" : "box",
						"rect" : [ 0.0, 26.0, 887.0, 602.0 ],
						"gridsize" : [ 15.0, 15.0 ],
						"showontab" : 1,
						"boxes" : [ 							{
								"box" : 								{
									"bubble" : 1,
									"fontname" : "Lato Light",
									"id" : "obj-35",
									"maxclass" : "comment",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 154.0, 263.5, 120.0, 25.0 ],
									"text" : "Save file to disk"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-25",
									"maxclass" : "message",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 195.0, 382.0, 69.0, 22.0 ],
									"text" : "framecount"
								}

							}
, 							{
								"box" : 								{
									"bubble" : 1,
									"id" : "obj-13",
									"maxclass" : "comment",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 222.0, 222.0, 96.0, 24.0 ],
									"text" : "Look inside!"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-1",
									"maxclass" : "message",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 103.0, 265.0, 42.0, 22.0 ],
									"text" : "export"
								}

							}
, 							{
								"box" : 								{
									"bgcolor" : [ 0.9, 0.65, 0.05, 1.0 ],
									"fontname" : "Arial Bold",
									"hint" : "",
									"id" : "obj-10",
									"ignoreclick" : 1,
									"legacytextcolor" : 1,
									"maxclass" : "textbutton",
									"numinlets" : 1,
									"numoutlets" : 3,
									"outlettype" : [ "", "", "int" ],
									"parameter_enable" : 0,
									"patching_rect" : [ 347.0, 166.0, 20.0, 20.0 ],
									"rounded" : 60.0,
									"saved_attribute_attributes" : 									{
										"bgcolor" : 										{
											"expression" : "themecolor.lesson_step_circle"
										}

									}
,
									"text" : "3",
									"textcolor" : [ 0.34902, 0.34902, 0.34902, 1.0 ]
								}

							}
, 							{
								"box" : 								{
									"bgcolor" : [ 0.9, 0.65, 0.05, 1.0 ],
									"fontname" : "Arial Bold",
									"hint" : "",
									"id" : "obj-9",
									"ignoreclick" : 1,
									"legacytextcolor" : 1,
									"maxclass" : "textbutton",
									"numinlets" : 1,
									"numoutlets" : 3,
									"outlettype" : [ "", "", "int" ],
									"parameter_enable" : 0,
									"patching_rect" : [ 548.0, 165.0, 20.0, 20.0 ],
									"rounded" : 60.0,
									"saved_attribute_attributes" : 									{
										"bgcolor" : 										{
											"expression" : "themecolor.lesson_step_circle"
										}

									}
,
									"text" : "2",
									"textcolor" : [ 0.34902, 0.34902, 0.34902, 1.0 ]
								}

							}
, 							{
								"box" : 								{
									"bubble" : 1,
									"bubblepoint" : 0.01,
									"bubbleside" : 2,
									"fontname" : "Lato Light",
									"id" : "obj-46",
									"maxclass" : "comment",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 575.5, 187.0, 140.0, 40.0 ],
									"text" : "Start the render context"
								}

							}
, 							{
								"box" : 								{
									"bgcolor" : [ 0.9, 0.65, 0.05, 1.0 ],
									"fontname" : "Arial Bold",
									"hint" : "",
									"id" : "obj-93",
									"ignoreclick" : 1,
									"legacytextcolor" : 1,
									"maxclass" : "textbutton",
									"numinlets" : 1,
									"numoutlets" : 3,
									"outlettype" : [ "", "", "int" ],
									"parameter_enable" : 0,
									"patching_rect" : [ 128.5, 140.0, 20.0, 20.0 ],
									"rounded" : 60.0,
									"saved_attribute_attributes" : 									{
										"bgcolor" : 										{
											"expression" : "themecolor.lesson_step_circle"
										}

									}
,
									"text" : "1",
									"textcolor" : [ 0.34902, 0.34902, 0.34902, 1.0 ]
								}

							}
, 							{
								"box" : 								{
									"bubble" : 1,
									"fontname" : "Lato Light",
									"id" : "obj-7",
									"linecount" : 2,
									"maxclass" : "comment",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 112.5, 167.0, 120.0, 39.0 ],
									"text" : "Click to genetare animated frames"
								}

							}
, 							{
								"box" : 								{
									"bubble" : 1,
									"fontname" : "Lato Light",
									"id" : "obj-6",
									"linecount" : 2,
									"maxclass" : "comment",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 442.5, 274.0, 97.0, 39.0 ],
									"text" : "Wrap around frame count"
								}

							}
, 							{
								"box" : 								{
									"bubble" : 1,
									"bubbleside" : 3,
									"fontname" : "Lato Light",
									"id" : "obj-5",
									"maxclass" : "comment",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 42.0, 338.0, 95.0, 25.0 ],
									"text" : "Frame count"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-14",
									"maxclass" : "number",
									"numinlets" : 1,
									"numoutlets" : 2,
									"outlettype" : [ "", "bang" ],
									"parameter_enable" : 0,
									"patching_rect" : [ 140.0, 339.0, 50.0, 22.0 ]
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-11",
									"maxclass" : "message",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 36.0, 265.0, 35.0, 22.0 ],
									"text" : "clear"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-8",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 2,
									"outlettype" : [ "appendframe", "" ],
									"patcher" : 									{
										"fileversion" : 1,
										"appversion" : 										{
											"major" : 9,
											"minor" : 0,
											"revision" : 6,
											"architecture" : "x64",
											"modernui" : 1
										}
,
										"classnamespace" : "box",
										"rect" : [ 189.0, 100.0, 988.0, 730.0 ],
										"gridsize" : [ 15.0, 15.0 ],
										"boxes" : [ 											{
												"box" : 												{
													"id" : "obj-15",
													"maxclass" : "newobj",
													"numinlets" : 6,
													"numoutlets" : 1,
													"outlettype" : [ "" ],
													"patching_rect" : [ 84.0, 430.0, 107.0, 22.0 ],
													"text" : "scale 0. 1. -1.8 2.4"
												}

											}
, 											{
												"box" : 												{
													"id" : "obj-16",
													"maxclass" : "message",
													"numinlets" : 2,
													"numoutlets" : 1,
													"outlettype" : [ "" ],
													"patching_rect" : [ 84.0, 462.0, 180.0, 22.0 ],
													"text" : "bezier -1. 0. -0.5 $1 0.5 -0.5 1. 0"
												}

											}
, 											{
												"box" : 												{
													"id" : "obj-14",
													"maxclass" : "newobj",
													"numinlets" : 1,
													"numoutlets" : 1,
													"outlettype" : [ "" ],
													"patching_rect" : [ 878.0, 515.0, 54.0, 22.0 ],
													"text" : "deferlow"
												}

											}
, 											{
												"box" : 												{
													"comment" : "",
													"id" : "obj-2",
													"index" : 2,
													"maxclass" : "outlet",
													"numinlets" : 1,
													"numoutlets" : 0,
													"patching_rect" : [ 878.0, 568.0, 30.0, 30.0 ]
												}

											}
, 											{
												"box" : 												{
													"bubble" : 1,
													"fontname" : "Lato Light",
													"id" : "obj-12",
													"linecount" : 4,
													"maxclass" : "comment",
													"numinlets" : 1,
													"numoutlets" : 0,
													"patching_rect" : [ 455.0, 380.0, 137.0, 68.0 ],
													"text" : "...and finally add some gradually changing shapes to each frame!"
												}

											}
, 											{
												"box" : 												{
													"bubble" : 1,
													"fontname" : "Lato Light",
													"id" : "obj-10",
													"linecount" : 2,
													"maxclass" : "comment",
													"numinlets" : 1,
													"numoutlets" : 0,
													"patching_rect" : [ 634.0, 334.0, 137.0, 39.0 ],
													"text" : "...set the drawing color..."
												}

											}
, 											{
												"box" : 												{
													"bubble" : 1,
													"fontname" : "Lato Light",
													"id" : "obj-8",
													"linecount" : 2,
													"maxclass" : "comment",
													"numinlets" : 1,
													"numoutlets" : 0,
													"patching_rect" : [ 721.0, 256.0, 162.0, 39.0 ],
													"text" : "add an new frame for each number..."
												}

											}
, 											{
												"box" : 												{
													"bubble" : 1,
													"fontname" : "Lato Light",
													"id" : "obj-4",
													"linecount" : 2,
													"maxclass" : "comment",
													"numinlets" : 1,
													"numoutlets" : 0,
													"patching_rect" : [ 499.0, 144.0, 162.0, 39.0 ],
													"text" : "fold to count up to 1. and down agin to 0."
												}

											}
, 											{
												"box" : 												{
													"bubble" : 1,
													"fontname" : "Lato Light",
													"id" : "obj-3",
													"linecount" : 2,
													"maxclass" : "comment",
													"numinlets" : 1,
													"numoutlets" : 0,
													"patching_rect" : [ 415.0, 84.0, 162.0, 39.0 ],
													"text" : "generate number from 0. to 2. in 100 steps"
												}

											}
, 											{
												"box" : 												{
													"id" : "obj-112",
													"maxclass" : "newobj",
													"numinlets" : 2,
													"numoutlets" : 1,
													"outlettype" : [ "float" ],
													"patching_rect" : [ 207.0, 401.0, 36.0, 22.0 ],
													"text" : "+ 90."
												}

											}
, 											{
												"box" : 												{
													"id" : "obj-111",
													"maxclass" : "newobj",
													"numinlets" : 2,
													"numoutlets" : 1,
													"outlettype" : [ "float" ],
													"patching_rect" : [ 251.0, 371.0, 36.0, 22.0 ],
													"text" : "+ 60."
												}

											}
, 											{
												"box" : 												{
													"id" : "obj-108",
													"maxclass" : "newobj",
													"numinlets" : 2,
													"numoutlets" : 1,
													"outlettype" : [ "float" ],
													"patching_rect" : [ 300.0, 339.0, 36.0, 22.0 ],
													"text" : "+ 30."
												}

											}
, 											{
												"box" : 												{
													"id" : "obj-105",
													"maxclass" : "message",
													"numinlets" : 2,
													"numoutlets" : 1,
													"outlettype" : [ "" ],
													"patching_rect" : [ 207.0, 433.0, 123.0, 22.0 ],
													"text" : "circle 0. 0. 0.6 90. $1."
												}

											}
, 											{
												"box" : 												{
													"id" : "obj-104",
													"maxclass" : "message",
													"numinlets" : 2,
													"numoutlets" : 1,
													"outlettype" : [ "" ],
													"patching_rect" : [ 251.0, 403.0, 123.0, 22.0 ],
													"text" : "circle 0. 0. 0.7 60. $1."
												}

											}
, 											{
												"box" : 												{
													"id" : "obj-91",
													"maxclass" : "message",
													"numinlets" : 2,
													"numoutlets" : 1,
													"outlettype" : [ "" ],
													"patching_rect" : [ 300.0, 371.0, 123.0, 22.0 ],
													"text" : "circle 0. 0. 0.8 30. $1."
												}

											}
, 											{
												"box" : 												{
													"id" : "obj-18",
													"maxclass" : "newobj",
													"numinlets" : 2,
													"numoutlets" : 1,
													"outlettype" : [ "float" ],
													"patching_rect" : [ 554.0, 278.0, 29.5, 22.0 ],
													"text" : "!- 1."
												}

											}
, 											{
												"box" : 												{
													"id" : "obj-17",
													"maxclass" : "newobj",
													"numinlets" : 2,
													"numoutlets" : 1,
													"outlettype" : [ "float" ],
													"patching_rect" : [ 355.0, 119.0, 32.0, 22.0 ],
													"text" : "/ 50."
												}

											}
, 											{
												"box" : 												{
													"id" : "obj-11",
													"maxclass" : "newobj",
													"numinlets" : 2,
													"numoutlets" : 3,
													"outlettype" : [ "bang", "bang", "int" ],
													"patching_rect" : [ 317.0, 84.0, 57.0, 22.0 ],
													"text" : "uzi 101 0"
												}

											}
, 											{
												"box" : 												{
													"id" : "obj-13",
													"maxclass" : "newobj",
													"numinlets" : 1,
													"numoutlets" : 2,
													"outlettype" : [ "float", "float" ],
													"patching_rect" : [ 355.0, 243.0, 176.0, 22.0 ],
													"text" : "t f f"
												}

											}
, 											{
												"box" : 												{
													"id" : "obj-82",
													"maxclass" : "message",
													"numinlets" : 2,
													"numoutlets" : 1,
													"outlettype" : [ "" ],
													"patching_rect" : [ 355.0, 336.0, 113.0, 22.0 ],
													"text" : "circle 0. 0. 0.9 0. $1"
												}

											}
, 											{
												"box" : 												{
													"id" : "obj-9",
													"maxclass" : "newobj",
													"numinlets" : 2,
													"numoutlets" : 1,
													"outlettype" : [ "" ],
													"patching_rect" : [ 512.0, 309.0, 61.0, 22.0 ],
													"text" : "pack 0. 0."
												}

											}
, 											{
												"box" : 												{
													"id" : "obj-6",
													"maxclass" : "message",
													"numinlets" : 2,
													"numoutlets" : 1,
													"outlettype" : [ "" ],
													"patching_rect" : [ 512.0, 336.0, 107.0, 22.0 ],
													"text" : "drawcolor 1. $2 $1"
												}

											}
, 											{
												"box" : 												{
													"id" : "obj-87",
													"maxclass" : "newobj",
													"numinlets" : 2,
													"numoutlets" : 1,
													"outlettype" : [ "float" ],
													"patching_rect" : [ 355.0, 278.0, 40.0, 22.0 ],
													"text" : "* 360."
												}

											}
, 											{
												"box" : 												{
													"id" : "obj-98",
													"maxclass" : "newobj",
													"numinlets" : 1,
													"numoutlets" : 2,
													"outlettype" : [ "float", "appendframe" ],
													"patching_rect" : [ 355.0, 216.0, 354.0, 22.0 ],
													"text" : "t f appendframe"
												}

											}
, 											{
												"box" : 												{
													"id" : "obj-5",
													"maxclass" : "newobj",
													"numinlets" : 3,
													"numoutlets" : 1,
													"outlettype" : [ "" ],
													"patching_rect" : [ 355.0, 153.0, 130.0, 22.0 ],
													"text" : "pong 0. 1. @mode fold"
												}

											}
, 											{
												"box" : 												{
													"comment" : "",
													"id" : "obj-1",
													"index" : 1,
													"maxclass" : "inlet",
													"numinlets" : 0,
													"numoutlets" : 1,
													"outlettype" : [ "bang" ],
													"patching_rect" : [ 317.0, 39.0, 30.0, 30.0 ]
												}

											}
, 											{
												"box" : 												{
													"comment" : "",
													"id" : "obj-7",
													"index" : 1,
													"maxclass" : "outlet",
													"numinlets" : 1,
													"numoutlets" : 0,
													"patching_rect" : [ 429.0, 568.0, 30.0, 30.0 ]
												}

											}
 ],
										"lines" : [ 											{
												"patchline" : 												{
													"destination" : [ "obj-11", 0 ],
													"source" : [ "obj-1", 0 ]
												}

											}
, 											{
												"patchline" : 												{
													"destination" : [ "obj-7", 0 ],
													"midpoints" : [ 260.5, 551.0, 438.5, 551.0 ],
													"source" : [ "obj-104", 0 ]
												}

											}
, 											{
												"patchline" : 												{
													"destination" : [ "obj-7", 0 ],
													"midpoints" : [ 216.5, 551.02734375, 438.5, 551.02734375 ],
													"source" : [ "obj-105", 0 ]
												}

											}
, 											{
												"patchline" : 												{
													"destination" : [ "obj-91", 0 ],
													"source" : [ "obj-108", 0 ]
												}

											}
, 											{
												"patchline" : 												{
													"destination" : [ "obj-14", 0 ],
													"midpoints" : [ 326.5, 200.94921875, 887.5, 200.94921875 ],
													"source" : [ "obj-11", 0 ]
												}

											}
, 											{
												"patchline" : 												{
													"destination" : [ "obj-17", 0 ],
													"source" : [ "obj-11", 2 ]
												}

											}
, 											{
												"patchline" : 												{
													"destination" : [ "obj-104", 0 ],
													"source" : [ "obj-111", 0 ]
												}

											}
, 											{
												"patchline" : 												{
													"destination" : [ "obj-105", 0 ],
													"source" : [ "obj-112", 0 ]
												}

											}
, 											{
												"patchline" : 												{
													"destination" : [ "obj-15", 0 ],
													"order" : 1,
													"source" : [ "obj-13", 0 ]
												}

											}
, 											{
												"patchline" : 												{
													"destination" : [ "obj-18", 0 ],
													"order" : 0,
													"source" : [ "obj-13", 1 ]
												}

											}
, 											{
												"patchline" : 												{
													"destination" : [ "obj-87", 0 ],
													"order" : 0,
													"source" : [ "obj-13", 0 ]
												}

											}
, 											{
												"patchline" : 												{
													"destination" : [ "obj-9", 0 ],
													"order" : 1,
													"source" : [ "obj-13", 1 ]
												}

											}
, 											{
												"patchline" : 												{
													"destination" : [ "obj-2", 0 ],
													"source" : [ "obj-14", 0 ]
												}

											}
, 											{
												"patchline" : 												{
													"destination" : [ "obj-16", 0 ],
													"source" : [ "obj-15", 0 ]
												}

											}
, 											{
												"patchline" : 												{
													"destination" : [ "obj-7", 0 ],
													"midpoints" : [ 93.5, 551.375, 438.5, 551.375 ],
													"source" : [ "obj-16", 0 ]
												}

											}
, 											{
												"patchline" : 												{
													"destination" : [ "obj-5", 0 ],
													"source" : [ "obj-17", 0 ]
												}

											}
, 											{
												"patchline" : 												{
													"destination" : [ "obj-9", 1 ],
													"source" : [ "obj-18", 0 ]
												}

											}
, 											{
												"patchline" : 												{
													"destination" : [ "obj-98", 0 ],
													"source" : [ "obj-5", 0 ]
												}

											}
, 											{
												"patchline" : 												{
													"destination" : [ "obj-7", 0 ],
													"midpoints" : [ 521.5, 552.06640625, 438.5, 552.06640625 ],
													"source" : [ "obj-6", 0 ]
												}

											}
, 											{
												"patchline" : 												{
													"destination" : [ "obj-7", 0 ],
													"midpoints" : [ 364.5, 549.97265625, 438.5, 549.97265625 ],
													"source" : [ "obj-82", 0 ]
												}

											}
, 											{
												"patchline" : 												{
													"destination" : [ "obj-108", 0 ],
													"order" : 1,
													"source" : [ "obj-87", 0 ]
												}

											}
, 											{
												"patchline" : 												{
													"destination" : [ "obj-111", 0 ],
													"order" : 2,
													"source" : [ "obj-87", 0 ]
												}

											}
, 											{
												"patchline" : 												{
													"destination" : [ "obj-112", 0 ],
													"order" : 3,
													"source" : [ "obj-87", 0 ]
												}

											}
, 											{
												"patchline" : 												{
													"destination" : [ "obj-82", 0 ],
													"order" : 0,
													"source" : [ "obj-87", 0 ]
												}

											}
, 											{
												"patchline" : 												{
													"destination" : [ "obj-6", 0 ],
													"source" : [ "obj-9", 0 ]
												}

											}
, 											{
												"patchline" : 												{
													"destination" : [ "obj-7", 0 ],
													"midpoints" : [ 309.5, 550.51171875, 438.5, 550.51171875 ],
													"source" : [ "obj-91", 0 ]
												}

											}
, 											{
												"patchline" : 												{
													"destination" : [ "obj-13", 0 ],
													"source" : [ "obj-98", 0 ]
												}

											}
, 											{
												"patchline" : 												{
													"destination" : [ "obj-7", 0 ],
													"midpoints" : [ 699.5, 551.74609375, 438.5, 551.74609375 ],
													"source" : [ "obj-98", 1 ]
												}

											}
 ]
									}
,
									"patching_rect" : [ 76.0, 223.0, 138.0, 22.0 ],
									"text" : "p make_animation"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-20",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 5,
									"outlettype" : [ "", "", "int", "int", "list" ],
									"patching_rect" : [ 75.5, 308.0, 102.0, 22.0 ],
									"text" : "jam.ilda.compose"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-17",
									"maxclass" : "button",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "bang" ],
									"parameter_enable" : 0,
									"patching_rect" : [ 75.5, 180.0, 24.0, 24.0 ]
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-28",
									"maxclass" : "message",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 569.5, 269.0, 215.0, 22.0 ],
									"text" : "visible $1, floating $1, size 330 210, $1"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-19",
									"maxclass" : "toggle",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "int" ],
									"parameter_enable" : 0,
									"patching_rect" : [ 569.5, 235.0, 24.0, 24.0 ]
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-4",
									"linecount" : 2,
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 3,
									"outlettype" : [ "jit_matrix", "bang", "" ],
									"patching_rect" : [ 569.5, 299.0, 193.0, 35.0 ],
									"text" : "jit.world ilda_comp_anim @visible 0 @erase_color 0. 0. 0. 1."
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-30",
									"maxclass" : "toggle",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "int" ],
									"parameter_enable" : 0,
									"patching_rect" : [ 335.5, 216.0, 24.0, 24.0 ]
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-26",
									"maxclass" : "newobj",
									"numinlets" : 5,
									"numoutlets" : 4,
									"outlettype" : [ "int", "", "", "int" ],
									"patching_rect" : [ 335.5, 282.0, 61.0, 22.0 ],
									"text" : "counter"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-24",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "int" ],
									"patching_rect" : [ 403.5, 282.0, 29.5, 22.0 ],
									"text" : "- 1"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-22",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "int" ],
									"patching_rect" : [ 335.5, 310.0, 87.0, 22.0 ],
									"text" : "% 100"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-21",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "bang" ],
									"patching_rect" : [ 335.5, 254.0, 63.0, 22.0 ],
									"text" : "qmetro 40"
								}

							}
, 							{
								"box" : 								{
									"bubble" : 1,
									"bubblepoint" : 1.0,
									"fontname" : "Lato Light",
									"id" : "obj-32",
									"linecount" : 2,
									"maxclass" : "comment",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 368.5, 190.0, 130.0, 39.0 ],
									"text" : "Start metro to cycle through frames"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-27",
									"maxclass" : "number",
									"numinlets" : 1,
									"numoutlets" : 2,
									"outlettype" : [ "", "bang" ],
									"parameter_enable" : 0,
									"patching_rect" : [ 335.5, 347.0, 50.0, 22.0 ]
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-23",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 2,
									"outlettype" : [ "", "" ],
									"patching_rect" : [ 311.5, 462.0, 99.0, 22.0 ],
									"text" : "route framecount"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-3",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 2,
									"outlettype" : [ "dictionary", "" ],
									"patching_rect" : [ 75.5, 426.0, 255.0, 22.0 ],
									"text" : "jam.jit.gl.ilda.sketch @drawto ilda_comp_anim"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-16",
									"maxclass" : "message",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 335.5, 377.0, 56.0, 22.0 ],
									"text" : "frame $1"
								}

							}
, 							{
								"box" : 								{
									"fontname" : "Lato Light",
									"id" : "obj-2",
									"maxclass" : "comment",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 10.0, 106.599853515625, 358.0, 21.0 ],
									"text" : "A simple example how to create animations of grometric shapes"
								}

							}
, 							{
								"box" : 								{
									"fontname" : "Lato Light",
									"fontsize" : 24.0,
									"id" : "obj-18",
									"maxclass" : "comment",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 10.0, 69.599853515625, 254.0, 35.0 ],
									"text" : "Creating Animations"
								}

							}
, 							{
								"box" : 								{
									"border" : 0,
									"filename" : "helpname.js",
									"id" : "obj-12",
									"ignoreclick" : 1,
									"jsarguments" : [ "jam.ilda.compose" ],
									"maxclass" : "jsui",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"parameter_enable" : 0,
									"patching_rect" : [ 10.0, 10.0, 365.527984619140625, 57.599853515625 ]
								}

							}
 ],
						"lines" : [ 							{
								"patchline" : 								{
									"destination" : [ "obj-20", 0 ],
									"source" : [ "obj-1", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-20", 0 ],
									"source" : [ "obj-11", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-3", 0 ],
									"midpoints" : [ 345.0, 415.5, 85.0, 415.5 ],
									"source" : [ "obj-16", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-8", 0 ],
									"source" : [ "obj-17", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-28", 0 ],
									"source" : [ "obj-19", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-14", 0 ],
									"source" : [ "obj-20", 3 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-3", 0 ],
									"source" : [ "obj-20", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-26", 0 ],
									"source" : [ "obj-21", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-27", 0 ],
									"source" : [ "obj-22", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-24", 0 ],
									"midpoints" : [ 321.0, 496.44921875, 445.58203125, 496.44921875, 445.58203125, 278.5859375, 413.0, 278.5859375 ],
									"source" : [ "obj-23", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-22", 1 ],
									"source" : [ "obj-24", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-3", 0 ],
									"source" : [ "obj-25", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-22", 0 ],
									"source" : [ "obj-26", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-16", 0 ],
									"source" : [ "obj-27", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-4", 0 ],
									"source" : [ "obj-28", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-23", 0 ],
									"source" : [ "obj-3", 1 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-21", 0 ],
									"source" : [ "obj-30", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-20", 0 ],
									"source" : [ "obj-8", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-25", 0 ],
									"source" : [ "obj-8", 1 ]
								}

							}
 ]
					}
,
					"patching_rect" : [ 197.0, 232.0, 131.0, 22.0 ],
					"text" : "p \"creating animations\""
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-4",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 0,
					"patcher" : 					{
						"fileversion" : 1,
						"appversion" : 						{
							"major" : 9,
							"minor" : 0,
							"revision" : 6,
							"architecture" : "x64",
							"modernui" : 1
						}
,
						"classnamespace" : "box",
						"rect" : [ 0.0, 26.0, 887.0, 602.0 ],
						"gridsize" : [ 15.0, 15.0 ],
						"showontab" : 1,
						"boxes" : [ 							{
								"box" : 								{
									"arrows" : 1,
									"id" : "obj-39",
									"justification" : 4,
									"maxclass" : "live.line",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 311.0, 213.0, 20.0, 24.0 ]
								}

							}
, 							{
								"box" : 								{
									"arrows" : 2,
									"id" : "obj-34",
									"justification" : 3,
									"maxclass" : "live.line",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 340.0, 213.0, 20.0, 24.0 ]
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-32",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 278.0, 344.0, 73.0, 22.0 ],
									"text" : "pak 1. 1."
								}

							}
, 							{
								"box" : 								{
									"format" : 6,
									"id" : "obj-30",
									"maxclass" : "flonum",
									"numinlets" : 1,
									"numoutlets" : 2,
									"outlettype" : [ "", "bang" ],
									"parameter_enable" : 0,
									"patching_rect" : [ 332.0, 320.0, 50.0, 22.0 ]
								}

							}
, 							{
								"box" : 								{
									"format" : 6,
									"id" : "obj-21",
									"maxclass" : "flonum",
									"numinlets" : 1,
									"numoutlets" : 2,
									"outlettype" : [ "", "bang" ],
									"parameter_enable" : 0,
									"patching_rect" : [ 277.0, 320.0, 50.0, 22.0 ]
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-23",
									"maxclass" : "message",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 276.0, 368.0, 101.0, 22.0 ],
									"text" : "scaleframe $1 $2"
								}

							}
, 							{
								"box" : 								{
									"hidden" : 1,
									"id" : "obj-15",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 465.0, 45.599853515625, 73.0, 22.0 ],
									"text" : "loadmess 1."
								}

							}
, 							{
								"box" : 								{
									"bgcolor" : [ 0.9, 0.65, 0.05, 1.0 ],
									"fontname" : "Arial Bold",
									"hint" : "",
									"id" : "obj-14",
									"ignoreclick" : 1,
									"legacytextcolor" : 1,
									"maxclass" : "textbutton",
									"numinlets" : 1,
									"numoutlets" : 3,
									"outlettype" : [ "", "", "int" ],
									"parameter_enable" : 0,
									"patching_rect" : [ 325.0, 191.0, 20.0, 20.0 ],
									"rounded" : 60.0,
									"saved_attribute_attributes" : 									{
										"bgcolor" : 										{
											"expression" : "themecolor.lesson_step_circle"
										}

									}
,
									"text" : "4",
									"textcolor" : [ 0.34902, 0.34902, 0.34902, 1.0 ]
								}

							}
, 							{
								"box" : 								{
									"bgcolor" : [ 0.9, 0.65, 0.05, 1.0 ],
									"fontname" : "Arial Bold",
									"hint" : "",
									"id" : "obj-9",
									"ignoreclick" : 1,
									"legacytextcolor" : 1,
									"maxclass" : "textbutton",
									"numinlets" : 1,
									"numoutlets" : 3,
									"outlettype" : [ "", "", "int" ],
									"parameter_enable" : 0,
									"patching_rect" : [ 136.0, 238.0, 20.0, 20.0 ],
									"rounded" : 60.0,
									"saved_attribute_attributes" : 									{
										"bgcolor" : 										{
											"expression" : "themecolor.lesson_step_circle"
										}

									}
,
									"text" : "3",
									"textcolor" : [ 0.34902, 0.34902, 0.34902, 1.0 ]
								}

							}
, 							{
								"box" : 								{
									"bubble" : 1,
									"bubbleside" : 2,
									"fontname" : "Lato Light",
									"id" : "obj-13",
									"linecount" : 2,
									"maxclass" : "comment",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 70.0, 258.0, 78.0, 54.0 ],
									"text" : "Select frame to edit"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-2",
									"maxclass" : "message",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 496.0, 530.0, 215.0, 22.0 ],
									"text" : "visible $1, floating $1, size 330 210, $1"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-3",
									"maxclass" : "toggle",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "int" ],
									"parameter_enable" : 0,
									"patching_rect" : [ 496.0, 496.0, 24.0, 24.0 ]
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-7",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 3,
									"outlettype" : [ "jit_matrix", "bang", "" ],
									"patching_rect" : [ 496.0, 560.0, 335.0, 22.0 ],
									"text" : "jit.world ilda_comp_frame @visible 0 @erase_color 0. 0. 0. 1."
								}

							}
, 							{
								"box" : 								{
									"bgcolor" : [ 0.9, 0.65, 0.05, 1.0 ],
									"fontname" : "Arial Bold",
									"hint" : "",
									"id" : "obj-93",
									"ignoreclick" : 1,
									"legacytextcolor" : 1,
									"maxclass" : "textbutton",
									"numinlets" : 1,
									"numoutlets" : 3,
									"outlettype" : [ "", "", "int" ],
									"parameter_enable" : 0,
									"patching_rect" : [ 669.0, 471.0, 20.0, 20.0 ],
									"rounded" : 60.0,
									"saved_attribute_attributes" : 									{
										"bgcolor" : 										{
											"expression" : "themecolor.lesson_step_circle"
										}

									}
,
									"text" : "1",
									"textcolor" : [ 0.34902, 0.34902, 0.34902, 1.0 ]
								}

							}
, 							{
								"box" : 								{
									"bubble" : 1,
									"fontname" : "Lato Light",
									"id" : "obj-46",
									"maxclass" : "comment",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 534.0, 495.5, 155.0, 25.0 ],
									"text" : "Start the render context"
								}

							}
, 							{
								"box" : 								{
									"bubble" : 1,
									"bubbleside" : 2,
									"fontname" : "Lato Light",
									"id" : "obj-38",
									"linecount" : 2,
									"maxclass" : "comment",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 354.0, 237.0, 114.0, 54.0 ],
									"text" : "Rotate currently selected edit-frame"
								}

							}
, 							{
								"box" : 								{
									"bubble" : 1,
									"bubblepoint" : 0.54,
									"bubbleside" : 2,
									"fontname" : "Lato Light",
									"id" : "obj-37",
									"linecount" : 4,
									"maxclass" : "comment",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 168.0, 237.0, 149.0, 83.0 ],
									"text" : "Scale currently selected edit-frame.\nUse 2 arguments to scale x and y axis separately"
								}

							}
, 							{
								"box" : 								{
									"bubble" : 1,
									"fontname" : "Lato Light",
									"id" : "obj-36",
									"maxclass" : "comment",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 567.0, 333.0, 238.0, 25.0 ],
									"text" : "Copies the frame at index 0 to index 3"
								}

							}
, 							{
								"box" : 								{
									"bubble" : 1,
									"fontname" : "Lato Light",
									"id" : "obj-33",
									"linecount" : 3,
									"maxclass" : "comment",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 566.0, 274.0, 292.0, 54.0 ],
									"text" : "Duplicatesthe  frame at index 0, \nif no argumenti s provided the currently selected edit-frame is duplicated"
								}

							}
, 							{
								"box" : 								{
									"bubble" : 1,
									"fontname" : "Lato Light",
									"id" : "obj-31",
									"linecount" : 3,
									"maxclass" : "comment",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 566.0, 214.0, 292.0, 54.0 ],
									"text" : "Removes the frame at index 0, \nif no argumenti s provided the currently selected edit-frame is removes"
								}

							}
, 							{
								"box" : 								{
									"bubble" : 1,
									"fontname" : "Lato Light",
									"id" : "obj-29",
									"maxclass" : "comment",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 566.0, 186.0, 164.0, 25.0 ],
									"text" : "Appends an empty frame"
								}

							}
, 							{
								"box" : 								{
									"bubble" : 1,
									"fontname" : "Lato Light",
									"id" : "obj-28",
									"maxclass" : "comment",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 566.0, 155.0, 164.0, 25.0 ],
									"text" : "Reverses the frame order"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-27",
									"maxclass" : "message",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 472.0, 334.0, 85.0, 22.0 ],
									"text" : "copyframe 0 3"
								}

							}
, 							{
								"box" : 								{
									"bgcolor" : [ 0.9, 0.65, 0.05, 1.0 ],
									"fontname" : "Arial Bold",
									"hint" : "",
									"id" : "obj-26",
									"ignoreclick" : 1,
									"legacytextcolor" : 1,
									"maxclass" : "textbutton",
									"numinlets" : 1,
									"numoutlets" : 3,
									"outlettype" : [ "", "", "int" ],
									"parameter_enable" : 0,
									"patching_rect" : [ 362.0, 121.0, 20.0, 20.0 ],
									"rounded" : 60.0,
									"saved_attribute_attributes" : 									{
										"bgcolor" : 										{
											"expression" : "themecolor.lesson_step_circle"
										}

									}
,
									"text" : "2",
									"textcolor" : [ 0.34902, 0.34902, 0.34902, 1.0 ]
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-25",
									"maxclass" : "message",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 11.0, 329.0, 35.0, 22.0 ],
									"text" : "clear"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-22",
									"maxclass" : "message",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 147.0, 205.0, 112.0, 22.0 ],
									"text" : "import fancy.ild 1"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-108",
									"maxclass" : "message",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 470.0, 290.0, 97.0, 22.0 ],
									"text" : "duplicateframe 0"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-87",
									"maxclass" : "message",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 470.0, 157.0, 85.0, 22.0 ],
									"text" : "reverseframes"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-70",
									"maxclass" : "message",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 470.0, 237.0, 89.0, 22.0 ],
									"text" : "removeframe 0"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-68",
									"maxclass" : "message",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 470.0, 186.0, 79.0, 22.0 ],
									"text" : "appendframe"
								}

							}
, 							{
								"box" : 								{
									"format" : 6,
									"id" : "obj-20",
									"maxclass" : "flonum",
									"numinlets" : 1,
									"numoutlets" : 2,
									"outlettype" : [ "", "bang" ],
									"parameter_enable" : 0,
									"patching_rect" : [ 390.0, 320.0, 50.0, 22.0 ]
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-19",
									"maxclass" : "message",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 390.0, 363.0, 87.0, 22.0 ],
									"text" : "rotateframe $1"
								}

							}
, 							{
								"box" : 								{
									"format" : 6,
									"id" : "obj-16",
									"maxclass" : "flonum",
									"numinlets" : 1,
									"numoutlets" : 2,
									"outlettype" : [ "", "bang" ],
									"parameter_enable" : 0,
									"patching_rect" : [ 182.0, 320.0, 50.0, 22.0 ]
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-11",
									"maxclass" : "message",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 182.0, 363.0, 84.0, 22.0 ],
									"text" : "scaleframe $1"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-10",
									"maxclass" : "number",
									"numinlets" : 1,
									"numoutlets" : 2,
									"outlettype" : [ "", "bang" ],
									"parameter_enable" : 0,
									"patching_rect" : [ 83.0, 325.0, 50.0, 22.0 ]
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-8",
									"maxclass" : "message",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 83.0, 363.0, 91.0, 22.0 ],
									"text" : "seteditframe $1"
								}

							}
, 							{
								"box" : 								{
									"bubble" : 1,
									"fontname" : "Lato Light",
									"id" : "obj-6",
									"maxclass" : "comment",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 136.0, 143.0, 246.0, 25.0 ],
									"text" : "Open an ILDA file and append the frames"
								}

							}
, 							{
								"box" : 								{
									"bubble" : 1,
									"bubbleside" : 3,
									"fontname" : "Lato Light",
									"id" : "obj-45",
									"maxclass" : "comment",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 66.0, 438.0, 165.0, 25.0 ],
									"text" : "Frame selected for editing"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-35",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 2,
									"outlettype" : [ "bang", "" ],
									"patching_rect" : [ 49.0, 488.0, 29.5, 22.0 ],
									"text" : "t b l"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-24",
									"maxclass" : "message",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 83.0, 488.0, 60.0, 22.0 ],
									"text" : "frame $1"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-17",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 2,
									"outlettype" : [ "dictionary", "" ],
									"patching_rect" : [ 49.0, 535.0, 259.0, 22.0 ],
									"text" : "jam.jit.gl.ilda.sketch @drawto ilda_comp_frame"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-4",
									"maxclass" : "message",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 49.0, 146.0, 88.0, 22.0 ],
									"text" : "import fancy.ild"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-1",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 2,
									"outlettype" : [ "", "list" ],
									"patching_rect" : [ 49.0, 177.0, 210.0, 22.0 ],
									"text" : "jam.ilda.file"
								}

							}
, 							{
								"box" : 								{
									"bubble" : 1,
									"fontname" : "Lato Light",
									"id" : "obj-44",
									"linecount" : 2,
									"maxclass" : "comment",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 378.0, 430.0, 99.0, 39.0 ],
									"text" : "Total number of frames"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-42",
									"maxclass" : "number",
									"numinlets" : 1,
									"numoutlets" : 2,
									"outlettype" : [ "", "bang" ],
									"parameter_enable" : 0,
									"patching_rect" : [ 325.0, 439.0, 50.0, 22.0 ]
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-40",
									"maxclass" : "number",
									"numinlets" : 1,
									"numoutlets" : 2,
									"outlettype" : [ "", "bang" ],
									"parameter_enable" : 0,
									"patching_rect" : [ 233.0, 439.0, 50.0, 22.0 ]
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-5",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 5,
									"outlettype" : [ "", "", "int", "int", "list" ],
									"patching_rect" : [ 49.0, 407.0, 388.0, 22.0 ],
									"text" : "jam.ilda.compose"
								}

							}
, 							{
								"box" : 								{
									"fontname" : "Lato Light",
									"fontsize" : 24.0,
									"id" : "obj-18",
									"maxclass" : "comment",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 10.0, 69.599853515625, 254.0, 35.0 ],
									"text" : "Manipulating Frames"
								}

							}
, 							{
								"box" : 								{
									"border" : 0,
									"filename" : "helpname.js",
									"id" : "obj-12",
									"ignoreclick" : 1,
									"jsarguments" : [ "jam.ilda.compose" ],
									"maxclass" : "jsui",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"parameter_enable" : 0,
									"patching_rect" : [ 10.0, 10.0, 365.527984619140625, 57.599853515625 ]
								}

							}
 ],
						"lines" : [ 							{
								"patchline" : 								{
									"destination" : [ "obj-22", 1 ],
									"source" : [ "obj-1", 1 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-5", 0 ],
									"source" : [ "obj-1", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-8", 0 ],
									"source" : [ "obj-10", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-5", 0 ],
									"midpoints" : [ 479.5, 400.458985046308953, 58.5, 400.458985046308953 ],
									"source" : [ "obj-108", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-5", 0 ],
									"midpoints" : [ 191.5, 399.557812545564957, 58.5, 399.557812545564957 ],
									"source" : [ "obj-11", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-16", 0 ],
									"hidden" : 1,
									"order" : 2,
									"source" : [ "obj-15", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-21", 0 ],
									"hidden" : 1,
									"order" : 1,
									"source" : [ "obj-15", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-30", 0 ],
									"hidden" : 1,
									"order" : 0,
									"source" : [ "obj-15", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-11", 0 ],
									"source" : [ "obj-16", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-5", 0 ],
									"midpoints" : [ 399.5, 399.059375038137659, 58.5, 399.059375038137659 ],
									"source" : [ "obj-19", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-7", 0 ],
									"source" : [ "obj-2", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-19", 0 ],
									"source" : [ "obj-20", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-32", 0 ],
									"source" : [ "obj-21", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-5", 0 ],
									"midpoints" : [ 285.5, 399.648437546915375, 58.5, 399.648437546915375 ],
									"source" : [ "obj-23", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-17", 0 ],
									"source" : [ "obj-24", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-5", 0 ],
									"source" : [ "obj-25", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-5", 0 ],
									"midpoints" : [ 481.5, 398.925130403542425, 58.5, 398.925130403542425 ],
									"source" : [ "obj-27", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-2", 0 ],
									"source" : [ "obj-3", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-32", 1 ],
									"source" : [ "obj-30", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-23", 0 ],
									"source" : [ "obj-32", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-17", 0 ],
									"source" : [ "obj-35", 1 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-17", 0 ],
									"source" : [ "obj-35", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-1", 0 ],
									"source" : [ "obj-4", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-24", 0 ],
									"midpoints" : [ 242.5, 471.859375, 92.5, 471.859375 ],
									"source" : [ "obj-40", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-35", 0 ],
									"source" : [ "obj-5", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-40", 0 ],
									"source" : [ "obj-5", 2 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-42", 0 ],
									"source" : [ "obj-5", 3 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-5", 0 ],
									"midpoints" : [ 479.5, 399.1647135981475, 58.5, 399.1647135981475 ],
									"source" : [ "obj-68", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-5", 0 ],
									"midpoints" : [ 479.5, 399.225911494751927, 58.5, 399.225911494751927 ],
									"source" : [ "obj-70", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-5", 0 ],
									"midpoints" : [ 92.5, 396.5, 58.5, 396.5 ],
									"source" : [ "obj-8", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-5", 0 ],
									"midpoints" : [ 479.5, 399.286406081402674, 58.5, 399.286406081402674 ],
									"source" : [ "obj-87", 0 ]
								}

							}
 ]
					}
,
					"patching_rect" : [ 153.0, 205.0, 135.0, 22.0 ],
					"text" : "p \"manipulating frames\""
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-3",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 0,
					"patcher" : 					{
						"fileversion" : 1,
						"appversion" : 						{
							"major" : 9,
							"minor" : 0,
							"revision" : 6,
							"architecture" : "x64",
							"modernui" : 1
						}
,
						"classnamespace" : "box",
						"rect" : [ 0.0, 26.0, 887.0, 602.0 ],
						"gridsize" : [ 15.0, 15.0 ],
						"showontab" : 1,
						"boxes" : [ 							{
								"box" : 								{
									"id" : "obj-1",
									"maxclass" : "message",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 424.0, 531.0, 215.0, 22.0 ],
									"text" : "visible $1, floating $1, size 330 210, $1"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-2",
									"maxclass" : "toggle",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "int" ],
									"parameter_enable" : 0,
									"patching_rect" : [ 424.0, 497.0, 24.0, 24.0 ]
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-3",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 3,
									"outlettype" : [ "jit_matrix", "bang", "" ],
									"patching_rect" : [ 424.0, 561.0, 324.0, 22.0 ],
									"text" : "jit.world ilda_comp_text @visible 0 @erase_color 0. 0. 0. 1."
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-43",
									"maxclass" : "message",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 10.0, 222.0, 35.0, 22.0 ],
									"text" : "clear"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-39",
									"maxclass" : "message",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 420.0, 217.0, 61.0, 22.0 ],
									"text" : "pen down"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-38",
									"maxclass" : "message",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 420.0, 188.0, 45.0, 22.0 ],
									"text" : "pen up"
								}

							}
, 							{
								"box" : 								{
									"bgcolor" : [ 0.9, 0.65, 0.05, 1.0 ],
									"fontname" : "Arial Bold",
									"hint" : "",
									"id" : "obj-34",
									"ignoreclick" : 1,
									"legacytextcolor" : 1,
									"maxclass" : "textbutton",
									"numinlets" : 1,
									"numoutlets" : 3,
									"outlettype" : [ "", "", "int" ],
									"parameter_enable" : 0,
									"patching_rect" : [ 356.5, 147.0, 20.0, 20.0 ],
									"rounded" : 60.0,
									"saved_attribute_attributes" : 									{
										"bgcolor" : 										{
											"expression" : "themecolor.lesson_step_circle"
										}

									}
,
									"text" : "5",
									"textcolor" : [ 0.34902, 0.34902, 0.34902, 1.0 ]
								}

							}
, 							{
								"box" : 								{
									"bubble" : 1,
									"bubbleside" : 2,
									"fontname" : "Lato Light",
									"id" : "obj-36",
									"maxclass" : "comment",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 310.5, 169.0, 66.0, 40.0 ],
									"text" : "Write text"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-33",
									"maxclass" : "message",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 296.0, 217.0, 95.0, 22.0 ],
									"text" : "text Laser Show"
								}

							}
, 							{
								"box" : 								{
									"bgcolor" : [ 0.9, 0.65, 0.05, 1.0 ],
									"fontname" : "Arial Bold",
									"hint" : "",
									"id" : "obj-31",
									"ignoreclick" : 1,
									"legacytextcolor" : 1,
									"maxclass" : "textbutton",
									"numinlets" : 1,
									"numoutlets" : 3,
									"outlettype" : [ "", "", "int" ],
									"parameter_enable" : 0,
									"patching_rect" : [ 229.0, 183.0, 20.0, 20.0 ],
									"rounded" : 60.0,
									"saved_attribute_attributes" : 									{
										"bgcolor" : 										{
											"expression" : "themecolor.lesson_step_circle"
										}

									}
,
									"text" : "4",
									"textcolor" : [ 0.34902, 0.34902, 0.34902, 1.0 ]
								}

							}
, 							{
								"box" : 								{
									"bubble" : 1,
									"fontname" : "Lato Light",
									"id" : "obj-30",
									"linecount" : 2,
									"maxclass" : "comment",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 145.0, 205.0, 104.0, 39.0 ],
									"text" : "set the writing position"
								}

							}
, 							{
								"box" : 								{
									"bgcolor" : [ 0.9, 0.65, 0.05, 1.0 ],
									"fontname" : "Arial Bold",
									"hint" : "",
									"id" : "obj-93",
									"ignoreclick" : 1,
									"legacytextcolor" : 1,
									"maxclass" : "textbutton",
									"numinlets" : 1,
									"numoutlets" : 3,
									"outlettype" : [ "", "", "int" ],
									"parameter_enable" : 0,
									"patching_rect" : [ 599.0, 474.0, 20.0, 20.0 ],
									"rounded" : 60.0,
									"saved_attribute_attributes" : 									{
										"bgcolor" : 										{
											"expression" : "themecolor.lesson_step_circle"
										}

									}
,
									"text" : "1",
									"textcolor" : [ 0.34902, 0.34902, 0.34902, 1.0 ]
								}

							}
, 							{
								"box" : 								{
									"bubble" : 1,
									"fontname" : "Lato Light",
									"id" : "obj-29",
									"linecount" : 3,
									"maxclass" : "comment",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 483.0, 185.0, 101.0, 54.0 ],
									"text" : "Move writing position one line up/down "
								}

							}
, 							{
								"box" : 								{
									"bgcolor" : [ 0.9, 0.65, 0.05, 1.0 ],
									"fontname" : "Arial Bold",
									"hint" : "",
									"id" : "obj-28",
									"ignoreclick" : 1,
									"legacytextcolor" : 1,
									"maxclass" : "textbutton",
									"numinlets" : 1,
									"numoutlets" : 3,
									"outlettype" : [ "", "", "int" ],
									"parameter_enable" : 0,
									"patching_rect" : [ 148.0, 406.0, 20.0, 20.0 ],
									"rounded" : 60.0,
									"saved_attribute_attributes" : 									{
										"bgcolor" : 										{
											"expression" : "themecolor.lesson_step_circle"
										}

									}
,
									"text" : "3",
									"textcolor" : [ 0.34902, 0.34902, 0.34902, 1.0 ]
								}

							}
, 							{
								"box" : 								{
									"bgcolor" : [ 0.9, 0.65, 0.05, 1.0 ],
									"fontname" : "Arial Bold",
									"hint" : "",
									"id" : "obj-26",
									"ignoreclick" : 1,
									"legacytextcolor" : 1,
									"maxclass" : "textbutton",
									"numinlets" : 1,
									"numoutlets" : 3,
									"outlettype" : [ "", "", "int" ],
									"parameter_enable" : 0,
									"patching_rect" : [ 194.0, 115.0, 20.0, 20.0 ],
									"rounded" : 60.0,
									"saved_attribute_attributes" : 									{
										"bgcolor" : 										{
											"expression" : "themecolor.lesson_step_circle"
										}

									}
,
									"text" : "2",
									"textcolor" : [ 0.34902, 0.34902, 0.34902, 1.0 ]
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-23",
									"maxclass" : "message",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 78.0, 217.0, 55.0, 22.0 ],
									"text" : "pen 0. 0."
								}

							}
, 							{
								"box" : 								{
									"attr" : "lineheight",
									"id" : "obj-21",
									"maxclass" : "attrui",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"parameter_enable" : 0,
									"patching_rect" : [ 607.0, 230.0, 150.0, 22.0 ]
								}

							}
, 							{
								"box" : 								{
									"attr" : "textalign",
									"id" : "obj-20",
									"maxclass" : "attrui",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"parameter_enable" : 0,
									"patching_rect" : [ 607.0, 206.0, 150.0, 22.0 ]
								}

							}
, 							{
								"box" : 								{
									"attr" : "textfontsize",
									"id" : "obj-19",
									"maxclass" : "attrui",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"parameter_enable" : 0,
									"patching_rect" : [ 607.0, 182.0, 150.0, 22.0 ]
								}

							}
, 							{
								"box" : 								{
									"fontname" : "Lato Light",
									"fontsize" : 24.0,
									"id" : "obj-18",
									"maxclass" : "comment",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 10.0, 70.0, 310.0, 35.0 ],
									"text" : "Writing text into a frame"
								}

							}
, 							{
								"box" : 								{
									"border" : 0,
									"filename" : "helpname.js",
									"id" : "obj-12",
									"ignoreclick" : 1,
									"jsarguments" : [ "jam.ilda.compose" ],
									"maxclass" : "jsui",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"parameter_enable" : 0,
									"patching_rect" : [ 10.0, 10.0, 365.527984619140625, 57.599853515625 ]
								}

							}
, 							{
								"box" : 								{
									"bubble" : 1,
									"fontname" : "Lato Light",
									"id" : "obj-46",
									"maxclass" : "comment",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 464.0, 496.0, 155.0, 25.0 ],
									"text" : "Start the render context"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-16",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 2,
									"outlettype" : [ "", "" ],
									"patching_rect" : [ 408.0, 387.0, 59.0, 22.0 ],
									"text" : "route font"
								}

							}
, 							{
								"box" : 								{
									"bubble" : 1,
									"fontname" : "Lato Light",
									"id" : "obj-15",
									"maxclass" : "comment",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 434.0, 421.0, 198.0, 25.0 ],
									"text" : "Font loading success/faliure"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-14",
									"linecount" : 2,
									"maxclass" : "message",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 285.0, 423.0, 144.0, 35.0 ],
									"text" : "\"Helvetica Light Oblique\" 1"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-9",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 113.0, 440.0, 76.0, 22.0 ],
									"text" : "prepend font"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-8",
									"items" : [ "Academy Engraved LET Plain:1.0", ",", "Adelle Sans Devanagari Bold", ",", "Adelle Sans Devanagari Extrabold", ",", "Adelle Sans Devanagari Heavy", ",", "Adelle Sans Devanagari Light", ",", "Adelle Sans Devanagari Regular", ",", "Adelle Sans Devanagari Semibold", ",", "Adelle Sans Devanagari Thin", ",", "AkayaKanadaka", ",", "AkayaTelivigala", ",", "Al Bayan Bold", ",", "Al Bayan Plain", ",", "Al Nile", ",", "Al Nile Bold", ",", "Al Tarikh Regular", ",", "American Typewriter", ",", "American Typewriter Bold", ",", "American Typewriter Condensed", ",", "American Typewriter Condensed Bold", ",", "American Typewriter Condensed Light", ",", "American Typewriter Light", ",", "American Typewriter Semibold", ",", "Andale Mono", ",", "Annai MN Regular", ",", "Apple Braille", ",", "Apple Braille Outline 6 Dot", ",", "Apple Braille Outline 8 Dot", ",", "Apple Braille Pinpoint 6 Dot", ",", "Apple Braille Pinpoint 8 Dot", ",", "Apple Chancery", ",", "Apple Color Emoji", ",", "Apple LiGothic Medium", ",", "Apple LiSung Light", ",", "Apple SD Gothic Neo Bold", ",", "Apple SD Gothic Neo ExtraBold", ",", "Apple SD Gothic Neo Heavy", ",", "Apple SD Gothic Neo Light", ",", "Apple SD Gothic Neo Medium", ",", "Apple SD Gothic Neo Regular", ",", "Apple SD Gothic Neo SemiBold", ",", "Apple SD Gothic Neo Thin", ",", "Apple SD Gothic Neo UltraLight", ",", "Apple Symbols", ",", "AppleGothic Regular", ",", "AppleMyungjo Regular", ",", "Arial", ",", "Arial Black", ",", "Arial Bold", ",", "Arial Bold Italic", ",", "Arial Hebrew", ",", "Arial Hebrew Bold", ",", "Arial Hebrew Light", ",", "Arial Hebrew Scholar", ",", "Arial Hebrew Scholar Bold", ",", "Arial Hebrew Scholar Light", ",", "Arial Italic", ",", "Arial Narrow", ",", "Arial Narrow Bold", ",", "Arial Narrow Bold Italic", ",", "Arial Narrow Italic", ",", "Arial Rounded MT Bold", ",", "Arial Unicode MS", ",", "Arima Koshi", ",", "Arima Koshi Black", ",", "Arima Koshi Bold", ",", "Arima Koshi ExtraBold", ",", "Arima Koshi ExtraLight", ",", "Arima Koshi Light", ",", "Arima Koshi Medium", ",", "Arima Koshi Thin", ",", "Arima Madurai", ",", "Arima Madurai Black", ",", "Arima Madurai Bold", ",", "Arima Madurai ExtraLight", ",", "Arima Madurai Light", ",", "Arima Madurai Medium", ",", "Arima Madurai Semi Bold", ",", "Arima Madurai Thin", ",", "Avenir Black", ",", "Avenir Black Oblique", ",", "Avenir Book", ",", "Avenir Book Oblique", ",", "Avenir Heavy", ",", "Avenir Heavy Oblique", ",", "Avenir Light", ",", "Avenir Light Oblique", ",", "Avenir Medium", ",", "Avenir Medium Oblique", ",", "Avenir Next Bold", ",", "Avenir Next Bold Italic", ",", "Avenir Next Condensed Bold", ",", "Avenir Next Condensed Bold Italic", ",", "Avenir Next Condensed Demi Bold", ",", "Avenir Next Condensed Demi Bold Italic", ",", "Avenir Next Condensed Heavy", ",", "Avenir Next Condensed Heavy Italic", ",", "Avenir Next Condensed Italic", ",", "Avenir Next Condensed Medium", ",", "Avenir Next Condensed Medium Italic", ",", "Avenir Next Condensed Regular", ",", "Avenir Next Condensed Ultra Light", ",", "Avenir Next Condensed Ultra Light Italic", ",", "Avenir Next Demi Bold", ",", "Avenir Next Demi Bold Italic", ",", "Avenir Next Heavy", ",", "Avenir Next Heavy Italic", ",", "Avenir Next Italic", ",", "Avenir Next Medium", ",", "Avenir Next Medium Italic", ",", "Avenir Next Regular", ",", "Avenir Next Ultra Light", ",", "Avenir Next Ultra Light Italic", ",", "Avenir Oblique", ",", "Avenir Roman", ",", "Ayuthaya", ",", "Baghdad Regular", ",", "Bai Jamjuree Bold", ",", "Bai Jamjuree Bold Italic", ",", "Bai Jamjuree ExtraLight", ",", "Bai Jamjuree ExtraLight Italic", ",", "Bai Jamjuree Italic", ",", "Bai Jamjuree Light", ",", "Bai Jamjuree Light Italic", ",", "Bai Jamjuree Medium", ",", "Bai Jamjuree Medium Italic", ",", "Bai Jamjuree Regular", ",", "Bai Jamjuree SemiBold", ",", "Bai Jamjuree SemiBold Italic", ",", "Baloo 2 Bold", ",", "Baloo 2 ExtraBold", ",", "Baloo 2 Medium", ",", "Baloo 2 Regular", ",", "Baloo 2 SemiBold", ",", "Baloo Bhai 2 Bold", ",", "Baloo Bhai 2 ExtraBold", ",", "Baloo Bhai 2 Medium", ",", "Baloo Bhai 2 Regular", ",", "Baloo Bhai 2 SemiBold", ",", "Baloo Bhaijaan Regular", ",", "Baloo Bhaina 2 Bold", ",", "Baloo Bhaina 2 ExtraBold", ",", "Baloo Bhaina 2 Medium", ",", "Baloo Bhaina 2 Regular", ",", "Baloo Bhaina 2 SemiBold", ",", "Baloo Chettan 2 Bold", ",", "Baloo Chettan 2 ExtraBold", ",", "Baloo Chettan 2 Medium", ",", "Baloo Chettan 2 Regular", ",", "Baloo Chettan 2 SemiBold", ",", "Baloo Da 2 Bold", ",", "Baloo Da 2 ExtraBold", ",", "Baloo Da 2 Medium", ",", "Baloo Da 2 Regular", ",", "Baloo Da 2 SemiBold", ",", "Baloo Paaji 2 Bold", ",", "Baloo Paaji 2 ExtraBold", ",", "Baloo Paaji 2 Medium", ",", "Baloo Paaji 2 Regular", ",", "Baloo Paaji 2 SemiBold", ",", "Baloo Tamma 2 Bold", ",", "Baloo Tamma 2 ExtraBold", ",", "Baloo Tamma 2 Medium", ",", "Baloo Tamma 2 Regular", ",", "Baloo Tamma 2 SemiBold", ",", "Baloo Tammudu 2 Bold", ",", "Baloo Tammudu 2 ExtraBold", ",", "Baloo Tammudu 2 Medium", ",", "Baloo Tammudu 2 Regular", ",", "Baloo Tammudu 2 SemiBold", ",", "Baloo Thambi 2 Bold", ",", "Baloo Thambi 2 ExtraBold", ",", "Baloo Thambi 2 Medium", ",", "Baloo Thambi 2 Regular", ",", "Baloo Thambi 2 SemiBold", ",", "Bangla MN", ",", "Bangla MN Bold", ",", "Bangla Sangam MN", ",", "Bangla Sangam MN Bold", ",", "Baoli SC Regular", ",", "Baoli TC Regular", ",", "Baskerville", ",", "Baskerville Bold", ",", "Baskerville Bold Italic", ",", "Baskerville Italic", ",", "Baskerville SemiBold", ",", "Baskerville SemiBold Italic", ",", "Beirut Regular", ",", "BiauKaiHK Regular", ",", "BiauKaiTC Regular", ",", "Big Caslon Medium", ",", "Bodoni 72 Bold", ",", "Bodoni 72 Book", ",", "Bodoni 72 Book Italic", ",", "Bodoni 72 Oldstyle Bold", ",", "Bodoni 72 Oldstyle Book", ",", "Bodoni 72 Oldstyle Book Italic", ",", "Bodoni 72 Smallcaps Book", ",", "Bodoni Ornaments", ",", "Bradley Hand Bold", ",", "Brush Script MT Italic", ",", "Cambay Devanagari Bold", ",", "Cambay Devanagari Bold Oblique", ",", "Cambay Devanagari Oblique", ",", "Cambay Devanagari Regular", ",", "Chakra Petch Bold", ",", "Chakra Petch Bold Italic", ",", "Chakra Petch ExtraLight", ",", "Chakra Petch ExtraLight Italic", ",", "Chakra Petch Italic", ",", "Chakra Petch Light", ",", "Chakra Petch Light Italic", ",", "Chakra Petch Medium", ",", "Chakra Petch Medium Italic", ",", "Chakra Petch Regular", ",", "Chakra Petch SemiBold", ",", "Chakra Petch SemiBold Italic", ",", "Chalkboard", ",", "Chalkboard Bold", ",", "Chalkboard SE Bold", ",", "Chalkboard SE Light", ",", "Chalkboard SE Regular", ",", "Chalkduster", ",", "Charm Bold", ",", "Charm Regular", ",", "Charmonman Bold", ",", "Charmonman Regular", ",", "Charter Black", ",", "Charter Black Italic", ",", "Charter Bold", ",", "Charter Bold Italic", ",", "Charter Italic", ",", "Charter Roman", ",", "Cochin", ",", "Cochin Bold", ",", "Cochin Bold Italic", ",", "Cochin Italic", ",", "Comic Sans MS", ",", "Comic Sans MS Bold", ",", "Copperplate", ",", "Copperplate Bold", ",", "Copperplate Light", ",", "Corsiva Hebrew", ",", "Corsiva Hebrew Bold", ",", "Courier New", ",", "Courier New Bold", ",", "Courier New Bold Italic", ",", "Courier New Italic", ",", "DIN Alternate Bold", ",", "DIN Condensed Bold", ",", "Damascus Bold", ",", "Damascus Light", ",", "Damascus Medium", ",", "Damascus Regular", ",", "Damascus Semi Bold", ",", "DecoType Naskh Regular", ",", "DecoType Nastaleeq Urdu", ",", "DecoType Nastaleeq Urdu Bold", ",", "Devanagari MT", ",", "Devanagari MT Bold", ",", "Devanagari Sangam MN", ",", "Devanagari Sangam MN Bold", ",", "Didot", ",", "Didot Bold", ",", "Didot Italic", ",", "Diwan Kufi Regular", ",", "Diwan Thuluth Regular", ",", "Euphemia UCAS", ",", "Euphemia UCAS Bold", ",", "Euphemia UCAS Italic", ",", "Fahkwang Bold", ",", "Fahkwang Bold Italic", ",", "Fahkwang ExtraLight", ",", "Fahkwang ExtraLight Italic", ",", "Fahkwang Italic", ",", "Fahkwang Light", ",", "Fahkwang Light Italic", ",", "Fahkwang Medium", ",", "Fahkwang Medium Italic", ",", "Fahkwang Regular", ",", "Fahkwang SemiBold", ",", "Fahkwang SemiBold Italic", ",", "Farah Regular", ",", "Farisi Regular", ",", "Futura Bold", ",", "Futura Condensed ExtraBold", ",", "Futura Condensed Medium", ",", "Futura Medium", ",", "Futura Medium Italic", ",", "GB18030 Bitmap", ",", "Galvji", ",", "Galvji Bold", ",", "Galvji Bold Oblique", ",", "Galvji Oblique", ",", "Geeza Pro Bold", ",", "Geeza Pro Regular", ",", "Geneva", ",", "Georgia", ",", "Georgia Bold", ",", "Georgia Bold Italic", ",", "Georgia Italic", ",", "Gill Sans", ",", "Gill Sans Bold", ",", "Gill Sans Bold Italic", ",", "Gill Sans Italic", ",", "Gill Sans Light", ",", "Gill Sans Light Italic", ",", "Gill Sans SemiBold", ",", "Gill Sans SemiBold Italic", ",", "Gill Sans UltraBold", ",", "Gotu", ",", "Grantha Sangam MN Black", ",", "Grantha Sangam MN Bold", ",", "Grantha Sangam MN DemiBold", ",", "Grantha Sangam MN Light", ",", "Grantha Sangam MN Medium", ",", "Grantha Sangam MN Regular", ",", "Gujarati MT", ",", "Gujarati MT Bold", ",", "Gujarati Sangam MN", ",", "Gujarati Sangam MN Bold", ",", "GungSeo Regular", ",", "Gurmukhi MN", ",", "Gurmukhi MN Bold", ",", "Gurmukhi MT", ",", "Gurmukhi Sangam MN", ",", "Gurmukhi Sangam MN Bold", ",", "Hannotate SC Bold", ",", "Hannotate SC Regular", ",", "Hannotate TC Bold", ",", "Hannotate TC Regular", ",", "HanziPen SC Bold", ",", "HanziPen SC Regular", ",", "HanziPen TC Bold", ",", "HanziPen TC Regular", ",", "HeadLineA Regular", ",", "Hei Regular", ",", "Heiti SC Light", ",", "Heiti SC Medium", ",", "Heiti TC Light", ",", "Heiti TC Medium", ",", "Helvetica", ",", "Helvetica Bold", ",", "Helvetica Bold Oblique", ",", "Helvetica Light", ",", "Helvetica Light Oblique", ",", "Helvetica Neue", ",", "Helvetica Neue Bold", ",", "Helvetica Neue Bold Italic", ",", "Helvetica Neue Condensed Black", ",", "Helvetica Neue Condensed Bold", ",", "Helvetica Neue Italic", ",", "Helvetica Neue Light", ",", "Helvetica Neue Light Italic", ",", "Helvetica Neue Medium", ",", "Helvetica Neue Medium Italic", ",", "Helvetica Neue Thin", ",", "Helvetica Neue Thin Italic", ",", "Helvetica Neue UltraLight", ",", "Helvetica Neue UltraLight Italic", ",", "Helvetica Oblique", ",", "Herculanum", ",", "Hiragino Maru Gothic ProN W4", ",", "Hiragino Mincho ProN W3", ",", "Hiragino Mincho ProN W6", ",", "Hiragino Sans GB W3", ",", "Hiragino Sans GB W6", ",", "Hiragino Sans TC W3", ",", "Hiragino Sans TC W6", ",", "Hiragino Sans W0", ",", "Hiragino Sans W1", ",", "Hiragino Sans W2", ",", "Hiragino Sans W3", ",", "Hiragino Sans W4", ",", "Hiragino Sans W5", ",", "Hiragino Sans W6", ",", "Hiragino Sans W7", ",", "Hiragino Sans W8", ",", "Hiragino Sans W9", ",", "Hoefler Text", ",", "Hoefler Text Black", ",", "Hoefler Text Black Italic", ",", "Hoefler Text Italic", ",", "Hoefler Text Ornaments", ",", "ITF Devanagari Bold", ",", "ITF Devanagari Book", ",", "ITF Devanagari Demi", ",", "ITF Devanagari Light", ",", "ITF Devanagari Marathi Bold", ",", "ITF Devanagari Marathi Book", ",", "ITF Devanagari Marathi Demi", ",", "ITF Devanagari Marathi Light", ",", "ITF Devanagari Marathi Medium", ",", "ITF Devanagari Medium", ",", "Impact", ",", "InaiMathi", ",", "InaiMathi Bold", ",", "Jaini", ",", "Jaini Purva", ",", "K2D Bold", ",", "K2D Bold Italic", ",", "K2D ExtraBold", ",", "K2D ExtraBold Italic", ",", "K2D ExtraLight", ",", "K2D ExtraLight Italic", ",", "K2D Italic", ",", "K2D Light", ",", "K2D Light Italic", ",", "K2D Medium", ",", "K2D Medium Italic", ",", "K2D Regular", ",", "K2D SemiBold", ",", "K2D SemiBold Italic", ",", "K2D Thin", ",", "K2D Thin Italic", ",", "Kai Regular", ",", "Kailasa Bold", ",", "Kailasa Regular", ",", "Kaiti SC Black", ",", "Kaiti SC Bold", ",", "Kaiti SC Regular", ",", "Kaiti TC Black", ",", "Kaiti TC Bold", ",", "Kaiti TC Regular", ",", "Kannada MN", ",", "Kannada MN Bold", ",", "Kannada Sangam MN", ",", "Kannada Sangam MN Bold", ",", "Katari Black", ",", "Katari Black Italic", ",", "Katari Bold", ",", "Katari Bold Italic", ",", "Katari Italic", ",", "Katari Medium", ",", "Katari Medium Italic", ",", "Katari Regular", ",", "Kavivanar", ",", "Kefa Bold", ",", "Kefa Regular", ",", "Khmer MN", ",", "Khmer MN Bold", ",", "Khmer Sangam MN", ",", "Klee Demibold", ",", "Klee Medium", ",", "KoHo Bold", ",", "KoHo Bold Italic", ",", "KoHo ExtraLight", ",", "KoHo ExtraLight Italic", ",", "KoHo Italic", ",", "KoHo Light", ",", "KoHo Light Italic", ",", "KoHo Medium", ",", "KoHo Medium Italic", ",", "KoHo Regular", ",", "KoHo SemiBold", ",", "KoHo SemiBold Italic", ",", "Kodchasan Bold", ",", "Kodchasan Bold Italic", ",", "Kodchasan ExtraLight", ",", "Kodchasan ExtraLight Italic", ",", "Kodchasan Italic", ",", "Kodchasan Light", ",", "Kodchasan Light Italic", ",", "Kodchasan Medium", ",", "Kodchasan Medium Italic", ",", "Kodchasan Regular", ",", "Kodchasan SemiBold", ",", "Kodchasan SemiBold Italic", ",", "Kohinoor Bangla", ",", "Kohinoor Bangla Bold", ",", "Kohinoor Bangla Light", ",", "Kohinoor Bangla Medium", ",", "Kohinoor Bangla Semibold", ",", "Kohinoor Devanagari Bold", ",", "Kohinoor Devanagari Light", ",", "Kohinoor Devanagari Medium", ",", "Kohinoor Devanagari Regular", ",", "Kohinoor Devanagari Semibold", ",", "Kohinoor Gujarati Bold", ",", "Kohinoor Gujarati Light", ",", "Kohinoor Gujarati Medium", ",", "Kohinoor Gujarati Regular", ",", "Kohinoor Gujarati Semibold", ",", "Kohinoor Telugu", ",", "Kohinoor Telugu Bold", ",", "Kohinoor Telugu Light", ",", "Kohinoor Telugu Medium", ",", "Kohinoor Telugu Semibold", ",", "Kokonor Regular", ",", "Krub Bold", ",", "Krub Bold Italic", ",", "Krub ExtraLight", ",", "Krub ExtraLight Italic", ",", "Krub Italic", ",", "Krub Light", ",", "Krub Light Italic", ",", "Krub Medium", ",", "Krub Medium Italic", ",", "Krub Regular", ",", "Krub SemiBold", ",", "Krub SemiBold Italic", ",", "Krungthep", ",", "KufiStandardGK Regular", ",", "Lahore Gurmukhi Bold", ",", "Lahore Gurmukhi Light", ",", "Lahore Gurmukhi Medium", ",", "Lahore Gurmukhi Regular", ",", "Lahore Gurmukhi SemiBold", ",", "Lantinghei SC Demibold", ",", "Lantinghei SC Extralight", ",", "Lantinghei SC Heavy", ",", "Lantinghei TC Demibold", ",", "Lantinghei TC Extralight", ",", "Lantinghei TC Heavy", ",", "Lao MN", ",", "Lao MN Bold", ",", "Lao Sangam MN", ",", "Lato Black", ",", "Lato Black Italic", ",", "Lato Bold", ",", "Lato Bold Italic", ",", "Lato Hairline", ",", "Lato Hairline Italic", ",", "Lato Heavy", ",", "Lato Heavy Italic", ",", "Lato Italic", ",", "Lato Light", ",", "Lato Light Italic", ",", "Lato Medium", ",", "Lato Medium Italic", ",", "Lato Regular", ",", "Lato Semibold", ",", "Lato Semibold Italic", ",", "Lato Thin", ",", "Lato Thin Italic", ",", "Lava Devanagari Bold", ",", "Lava Devanagari Heavy", ",", "Lava Devanagari Medium", ",", "Lava Devanagari Regular", ",", "Lava Kannada Bold", ",", "Lava Kannada Heavy", ",", "Lava Kannada Medium", ",", "Lava Kannada Regular", ",", "Lava Telugu Bold", ",", "Lava Telugu Heavy", ",", "Lava Telugu Medium", ",", "Lava Telugu Regular", ",", "LiHei Pro", ",", "LiSong Pro", ",", "Libian SC Regular", ",", "Libian TC Regular", ",", "Lucida Grande", ",", "Lucida Grande Bold", ",", "Luminari", ",", "Maku Bold", ",", "Maku Regular", ",", "Malayalam MN", ",", "Malayalam MN Bold", ",", "Malayalam Sangam MN", ",", "Malayalam Sangam MN Bold", ",", "Mali Bold", ",", "Mali Bold Italic", ",", "Mali ExtraLight", ",", "Mali ExtraLight Italic", ",", "Mali Italic", ",", "Mali Light", ",", "Mali Light Italic", ",", "Mali Medium", ",", "Mali Medium Italic", ",", "Mali Regular", ",", "Mali SemiBold", ",", "Mali SemiBold Italic", ",", "Marker Felt Thin", ",", "Marker Felt Wide", ",", "Menlo Bold", ",", "Menlo Bold Italic", ",", "Menlo Italic", ",", "Menlo Regular", ",", "Microsoft Sans Serif", ",", "Mishafi Gold Regular", ",", "Mishafi Regular", ",", "Modak", ",", "Monaco", ",", "Mshtakan", ",", "Mshtakan Bold", ",", "Mshtakan BoldOblique", ",", "Mshtakan Oblique", ",", "Mukta Bold", ",", "Mukta ExtraBold", ",", "Mukta ExtraLight", ",", "Mukta Light", ",", "Mukta Malar Bold", ",", "Mukta Malar ExtraBold", ",", "Mukta Malar ExtraLight", ",", "Mukta Malar Light", ",", "Mukta Malar Medium", ",", "Mukta Malar Regular", ",", "Mukta Malar SemiBold", ",", "Mukta Medium", ",", "Mukta Regular", ",", "Mukta SemiBold", ",", "Mukta Vaani Bold", ",", "Mukta Vaani ExtraBold", ",", "Mukta Vaani ExtraLight", ",", "Mukta Vaani Light", ",", "Mukta Vaani Medium", ",", "Mukta Vaani Regular", ",", "Mukta Vaani SemiBold", ",", "MuktaMahee Bold", ",", "MuktaMahee ExtraBold", ",", "MuktaMahee ExtraLight", ",", "MuktaMahee Light", ",", "MuktaMahee Medium", ",", "MuktaMahee Regular", ",", "MuktaMahee SemiBold", ",", "Muna Black", ",", "Muna Bold", ",", "Muna Regular", ",", "Myanmar MN", ",", "Myanmar MN Bold", ",", "Myanmar Sangam MN", ",", "Myanmar Sangam MN Bold", ",", "Nadeem Regular", ",", "Nanum Brush Script", ",", "Nanum Pen Script", ",", "NanumGothic", ",", "NanumGothic Bold", ",", "NanumGothic ExtraBold", ",", "NanumMyeongjo", ",", "NanumMyeongjo Bold", ",", "NanumMyeongjo ExtraBold", ",", "New Peninim MT", ",", "New Peninim MT Bold", ",", "New Peninim MT Bold Inclined", ",", "New Peninim MT Inclined", ",", "Niramit Bold", ",", "Niramit Bold Italic", ",", "Niramit ExtraLight", ",", "Niramit ExtraLight Italic", ",", "Niramit Italic", ",", "Niramit Light", ",", "Niramit Light Italic", ",", "Niramit Medium", ",", "Niramit Medium Italic", ",", "Niramit Regular", ",", "Niramit SemiBold", ",", "Niramit SemiBold Italic", ",", "Noteworthy Bold", ",", "Noteworthy Light", ",", "Noto Nastaliq Urdu", ",", "Noto Nastaliq Urdu Bold", ",", "Noto Sans Batak Regular", ",", "Noto Sans Kannada Black", ",", "Noto Sans Kannada Bold", ",", "Noto Sans Kannada ExtraBold", ",", "Noto Sans Kannada ExtraLight", ",", "Noto Sans Kannada Light", ",", "Noto Sans Kannada Medium", ",", "Noto Sans Kannada Regular", ",", "Noto Sans Kannada SemiBold", ",", "Noto Sans Kannada Thin", ",", "Noto Sans Myanmar Black", ",", "Noto Sans Myanmar Bold", ",", "Noto Sans Myanmar ExtraBold", ",", "Noto Sans Myanmar ExtraLight", ",", "Noto Sans Myanmar Light", ",", "Noto Sans Myanmar Medium", ",", "Noto Sans Myanmar Regular", ",", "Noto Sans Myanmar SemiBold", ",", "Noto Sans Myanmar Thin", ",", "Noto Sans NKo Regular", ",", "Noto Sans Oriya", ",", "Noto Sans Oriya Bold", ",", "Noto Sans Syriac Regular Black", ",", "Noto Sans Syriac Regular Bold", ",", "Noto Sans Syriac Regular ExtraBold", ",", "Noto Sans Syriac Regular ExtraLight", ",", "Noto Sans Syriac Regular Light", ",", "Noto Sans Syriac Regular Medium", ",", "Noto Sans Syriac Regular Regular", ",", "Noto Sans Syriac Regular SemiBold", ",", "Noto Sans Syriac Regular Thin", ",", "Noto Sans Tagalog Regular", ",", "Noto Serif Kannada Black", ",", "Noto Serif Kannada Bold", ",", "Noto Serif Kannada ExtraBold", ",", "Noto Serif Kannada ExtraLight", ",", "Noto Serif Kannada Light", ",", "Noto Serif Kannada Medium", ",", "Noto Serif Kannada Regular", ",", "Noto Serif Kannada SemiBold", ",", "Noto Serif Kannada Thin", ",", "Noto Serif Myanmar Black", ",", "Noto Serif Myanmar Bold", ",", "Noto Serif Myanmar ExtraBold", ",", "Noto Serif Myanmar ExtraLight", ",", "Noto Serif Myanmar Light", ",", "Noto Serif Myanmar Medium", ",", "Noto Serif Myanmar Regular", ",", "Noto Serif Myanmar SemiBold", ",", "Noto Serif Myanmar Thin", ",", "October Compressed Devanagari Black", ",", "October Compressed Devanagari Bold", ",", "October Compressed Devanagari ExtraLight", ",", "October Compressed Devanagari Hairline", ",", "October Compressed Devanagari Heavy", ",", "October Compressed Devanagari Light", ",", "October Compressed Devanagari Medium", ",", "October Compressed Devanagari Regular", ",", "October Compressed Devanagari Thin", ",", "October Compressed Tamil Black", ",", "October Compressed Tamil Bold", ",", "October Compressed Tamil ExtraLight", ",", "October Compressed Tamil Hairline", ",", "October Compressed Tamil Heavy", ",", "October Compressed Tamil Light", ",", "October Compressed Tamil Medium", ",", "October Compressed Tamil Regular", ",", "October Compressed Tamil Thin", ",", "October Condensed Devanagari Black", ",", "October Condensed Devanagari Bold", ",", "October Condensed Devanagari ExtraLight", ",", "October Condensed Devanagari Hairline", ",", "October Condensed Devanagari Heavy", ",", "October Condensed Devanagari Light", ",", "October Condensed Devanagari Medium", ",", "October Condensed Devanagari Regular", ",", "October Condensed Devanagari Thin", ",", "October Condensed Tamil Black", ",", "October Condensed Tamil Bold", ",", "October Condensed Tamil ExtraLight", ",", "October Condensed Tamil Hairline", ",", "October Condensed Tamil Heavy", ",", "October Condensed Tamil Light", ",", "October Condensed Tamil Medium", ",", "October Condensed Tamil Regular", ",", "October Condensed Tamil Thin", ",", "October Devanagari Black", ",", "October Devanagari Bold", ",", "October Devanagari ExtraLight", ",", "October Devanagari Hairline", ",", "October Devanagari Heavy", ",", "October Devanagari Light", ",", "October Devanagari Medium", ",", "October Devanagari Regular", ",", "October Devanagari Thin", ",", "October Tamil Black", ",", "October Tamil Bold", ",", "October Tamil ExtraLight", ",", "October Tamil Hairline", ",", "October Tamil Heavy", ",", "October Tamil Light", ",", "October Tamil Medium", ",", "October Tamil Regular", ",", "October Tamil Thin", ",", "Optima Bold", ",", "Optima Bold Italic", ",", "Optima ExtraBlack", ",", "Optima Italic", ",", "Optima Regular", ",", "Oriya MN", ",", "Oriya MN Bold", ",", "Oriya Sangam MN", ",", "Oriya Sangam MN Bold", ",", "Osaka", ",", "Osaka-Mono", ",", "PCMyungjo Regular", ",", "PSL Ornanong Pro", ",", "PSL Ornanong Pro Bold", ",", "PSL Ornanong Pro Bold Italic", ",", "PSL Ornanong Pro Demibold", ",", "PSL Ornanong Pro Demibold Italic", ",", "PSL Ornanong Pro Italic", ",", "PSL Ornanong Pro Light", ",", "PSL Ornanong Pro Light Italic", ",", "PT Mono", ",", "PT Mono Bold", ",", "PT Sans", ",", "PT Sans Bold", ",", "PT Sans Bold Italic", ",", "PT Sans Caption", ",", "PT Sans Caption Bold", ",", "PT Sans Italic", ",", "PT Sans Narrow", ",", "PT Sans Narrow Bold", ",", "PT Serif", ",", "PT Serif Bold", ",", "PT Serif Bold Italic", ",", "PT Serif Caption", ",", "PT Serif Caption Italic", ",", "PT Serif Italic", ",", "Palatino", ",", "Palatino Bold", ",", "Palatino Bold Italic", ",", "Palatino Italic", ",", "Papyrus", ",", "Papyrus Condensed", ",", "Party LET Plain", ",", "Phosphate Inline", ",", "Phosphate Solid", ",", "PilGi Regular", ",", "PingFang HK Light", ",", "PingFang HK Medium", ",", "PingFang HK Regular", ",", "PingFang HK Semibold", ",", "PingFang HK Thin", ",", "PingFang HK Ultralight", ",", "PingFang MO Light", ",", "PingFang MO Medium", ",", "PingFang MO Regular", ",", "PingFang MO Semibold", ",", "PingFang MO Thin", ",", "PingFang MO Ultralight", ",", "PingFang SC Light", ",", "PingFang SC Medium", ",", "PingFang SC Regular", ",", "PingFang SC Semibold", ",", "PingFang SC Thin", ",", "PingFang SC Ultralight", ",", "PingFang TC Light", ",", "PingFang TC Medium", ",", "PingFang TC Regular", ",", "PingFang TC Semibold", ",", "PingFang TC Thin", ",", "PingFang TC Ultralight", ",", "Plantagenet Cherokee", ",", "Raanana", ",", "Raanana Bold", ",", "Rockwell", ",", "Rockwell Bold", ",", "Rockwell Bold Italic", ",", "Rockwell Italic", ",", "STFangsong", ",", "STHeiti", ",", "STIX Two Text Bold", ",", "STIX Two Text Italic Bold Italic", ",", "STIX Two Text Italic Italic", ",", "STIX Two Text Italic Medium Italic", ",", "STIX Two Text Italic SemiBold Italic", ",", "STIX Two Text Medium", ",", "STIX Two Text Regular", ",", "STIX Two Text SemiBold", ",", "STKaiti", ",", "STSong", ",", "STXihei", ",", "Sama Devanagari Bold", ",", "Sama Devanagari Book", ",", "Sama Devanagari ExtraBold", ",", "Sama Devanagari Medium", ",", "Sama Devanagari Regular", ",", "Sama Devanagari SemiBold", ",", "Sama Gujarati Bold", ",", "Sama Gujarati Book", ",", "Sama Gujarati ExtraBold", ",", "Sama Gujarati Medium", ",", "Sama Gujarati Regular", ",", "Sama Gujarati SemiBold", ",", "Sama Gurmukhi Bold", ",", "Sama Gurmukhi Book", ",", "Sama Gurmukhi ExtraBold", ",", "Sama Gurmukhi Medium", ",", "Sama Gurmukhi Regular", ",", "Sama Gurmukhi SemiBold", ",", "Sama Kannada Bold", ",", "Sama Kannada Book", ",", "Sama Kannada ExtraBold", ",", "Sama Kannada Medium", ",", "Sama Kannada Regular", ",", "Sama Kannada SemiBold", ",", "Sama Malayalam Bold", ",", "Sama Malayalam Book", ",", "Sama Malayalam ExtraBold", ",", "Sama Malayalam Medium", ",", "Sama Malayalam Regular", ",", "Sama Malayalam SemiBold", ",", "Sama Tamil Bold", ",", "Sama Tamil Book", ",", "Sama Tamil ExtraBold", ",", "Sama Tamil Medium", ",", "Sama Tamil Regular", ",", "Sama Tamil SemiBold", ",", "Sana Regular", ",", "Sarabun Bold", ",", "Sarabun Bold Italic", ",", "Sarabun ExtraBold", ",", "Sarabun ExtraBold Italic", ",", "Sarabun ExtraLight", ",", "Sarabun ExtraLight Italic", ",", "Sarabun Italic", ",", "Sarabun Light", ",", "Sarabun Light Italic", ",", "Sarabun Medium", ",", "Sarabun Medium Italic", ",", "Sarabun Regular", ",", "Sarabun SemiBold", ",", "Sarabun SemiBold Italic", ",", "Sarabun Thin", ",", "Sarabun Thin Italic", ",", "Sathu", ",", "Savoye LET Plain:1.0", ",", "Shobhika Bold", ",", "Shobhika Regular", ",", "Shree Devanagari 714", ",", "Shree Devanagari 714 Bold", ",", "Shree Devanagari 714 Bold Italic", ",", "Shree Devanagari 714 Italic", ",", "SignPainter-HouseScript", ",", "SignPainter-HouseScript Semibold", ",", "Silom", ",", "SimSong Bold", ",", "SimSong Regular", ",", "Sinhala MN", ",", "Sinhala MN Bold", ",", "Sinhala Sangam MN", ",", "Sinhala Sangam MN Bold", ",", "Skia Black", ",", "Skia Black Condensed", ",", "Skia Black Extended", ",", "Skia Bold", ",", "Skia Condensed", ",", "Skia Extended", ",", "Skia Light", ",", "Skia Light Condensed", ",", "Skia Light Extended", ",", "Skia Regular", ",", "Snell Roundhand", ",", "Snell Roundhand Black", ",", "Snell Roundhand Bold", ",", "Songti SC Black", ",", "Songti SC Bold", ",", "Songti SC Light", ",", "Songti SC Regular", ",", "Songti TC Bold", ",", "Songti TC Light", ",", "Songti TC Regular", ",", "Srisakdi Bold", ",", "Srisakdi Regular", ",", "Sukhumvit Set Bold", ",", "Sukhumvit Set Light", ",", "Sukhumvit Set Medium", ",", "Sukhumvit Set Semi Bold", ",", "Sukhumvit Set Text", ",", "Sukhumvit Set Thin", ",", "Symbol", ",", "Tahoma", ",", "Tahoma Bold", ",", "Tamil MN", ",", "Tamil MN Bold", ",", "Tamil Sangam MN", ",", "Tamil Sangam MN Black", ",", "Tamil Sangam MN Bold", ",", "Tamil Sangam MN Demibold", ",", "Tamil Sangam MN Light", ",", "Tamil Sangam MN Medium", ",", "Telugu MN", ",", "Telugu MN Bold", ",", "Telugu Sangam MN", ",", "Telugu Sangam MN Bold", ",", "Thonburi", ",", "Thonburi Bold", ",", "Thonburi Light", ",", "Times New Roman", ",", "Times New Roman Bold", ",", "Times New Roman Bold Italic", ",", "Times New Roman Italic", ",", "Tiro Bangla", ",", "Tiro Bangla Italic", ",", "Tiro Devanagari Hindi", ",", "Tiro Devanagari Hindi Italic", ",", "Tiro Devanagari Marathi", ",", "Tiro Devanagari Marathi Italic", ",", "Tiro Devanagari Sanskrit", ",", "Tiro Devanagari Sanskrit Italic", ",", "Tiro Gurmukhi", ",", "Tiro Gurmukhi Italic", ",", "Tiro Kannada", ",", "Tiro Kannada Italic", ",", "Tiro Tamil", ",", "Tiro Tamil Italic", ",", "Tiro Telugu", ",", "Tiro Telugu Italic", ",", "Toppan Bunkyu Gothic Demibold", ",", "Toppan Bunkyu Gothic Regular", ",", "Trattatello", ",", "Trebuchet MS", ",", "Trebuchet MS Bold", ",", "Trebuchet MS Bold Italic", ",", "Trebuchet MS Italic", ",", "Tsukushi A Round Gothic Bold", ",", "Tsukushi A Round Gothic Regular", ",", "Tsukushi B Round Gothic Bold", ",", "Tsukushi B Round Gothic Regular", ",", "Verdana", ",", "Verdana Bold", ",", "Verdana Bold Italic", ",", "Verdana Italic", ",", "Waseem Light", ",", "Waseem Regular", ",", "Webdings", ",", "Wingdings", ",", "Wingdings 2", ",", "Wingdings 3", ",", "Xingkai SC Bold", ",", "Xingkai SC Light", ",", "Xingkai TC Bold", ",", "Xingkai TC Light", ",", "YuKyokasho Bold", ",", "YuKyokasho Medium", ",", "YuKyokasho Yoko Bold", ",", "YuKyokasho Yoko Medium", ",", "YuMincho +36p Kana Demibold", ",", "YuMincho +36p Kana Extrabold", ",", "YuMincho +36p Kana Medium", ",", "YuMincho Demibold", ",", "YuMincho Extrabold", ",", "YuMincho Medium", ",", "Yuanti SC Bold", ",", "Yuanti SC Light", ",", "Yuanti SC Regular", ",", "Yuanti TC Bold", ",", "Yuanti TC Light", ",", "Yuanti TC Regular", ",", "Zapf Dingbats", ",", "Zapfino" ],
									"maxclass" : "umenu",
									"numinlets" : 1,
									"numoutlets" : 3,
									"outlettype" : [ "int", "", "" ],
									"parameter_enable" : 0,
									"patching_rect" : [ 53.0, 324.0, 140.0, 22.0 ]
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-4",
									"maxclass" : "message",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 39.0, 193.0, 51.0, 22.0 ],
									"text" : "getfonts"
								}

							}
, 							{
								"box" : 								{
									"bubble" : 1,
									"bubbleside" : 0,
									"fontname" : "Lato Light",
									"id" : "obj-45",
									"linecount" : 2,
									"maxclass" : "comment",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 194.0, 357.0, 100.0, 54.0 ],
									"text" : "Frame selected for editing"
								}

							}
, 							{
								"box" : 								{
									"bubble" : 1,
									"fontname" : "Lato Light",
									"id" : "obj-44",
									"linecount" : 2,
									"maxclass" : "comment",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 373.0, 320.0, 99.0, 39.0 ],
									"text" : "Total number of frames"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-42",
									"maxclass" : "number",
									"numinlets" : 1,
									"numoutlets" : 2,
									"outlettype" : [ "", "bang" ],
									"parameter_enable" : 0,
									"patching_rect" : [ 315.0, 324.0, 50.0, 22.0 ]
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-40",
									"maxclass" : "number",
									"numinlets" : 1,
									"numoutlets" : 2,
									"outlettype" : [ "", "bang" ],
									"parameter_enable" : 0,
									"patching_rect" : [ 223.5, 324.0, 50.0, 22.0 ]
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-35",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 2,
									"outlettype" : [ "bang", "" ],
									"patching_rect" : [ 39.0, 514.0, 29.5, 22.0 ],
									"text" : "t b l"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-24",
									"maxclass" : "message",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 73.0, 514.0, 60.0, 22.0 ],
									"text" : "frame $1"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-17",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 2,
									"outlettype" : [ "dictionary", "" ],
									"patching_rect" : [ 39.0, 561.0, 248.0, 22.0 ],
									"text" : "jam.jit.gl.ilda.sketch @drawto ilda_comp_text"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-5",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 5,
									"outlettype" : [ "", "", "int", "int", "list" ],
									"patching_rect" : [ 39.0, 273.0, 388.0, 22.0 ],
									"text" : "jam.ilda.compose"
								}

							}
, 							{
								"box" : 								{
									"bubble" : 1,
									"bubblepoint" : 0.01,
									"bubbleside" : 2,
									"fontname" : "Lato Light",
									"id" : "obj-25",
									"linecount" : 2,
									"maxclass" : "comment",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 87.0, 137.0, 128.0, 54.0 ],
									"text" : "Load compatible fonts list to umenu"
								}

							}
, 							{
								"box" : 								{
									"bubble" : 1,
									"bubbleside" : 0,
									"fontname" : "Lato Light",
									"id" : "obj-27",
									"maxclass" : "comment",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 66.0, 364.0, 102.0, 40.0 ],
									"text" : "Select font face"
								}

							}
 ],
						"lines" : [ 							{
								"patchline" : 								{
									"destination" : [ "obj-3", 0 ],
									"source" : [ "obj-1", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-14", 1 ],
									"source" : [ "obj-16", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-5", 0 ],
									"midpoints" : [ 616.5, 267.84765625, 48.5, 267.84765625 ],
									"source" : [ "obj-19", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-1", 0 ],
									"source" : [ "obj-2", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-5", 0 ],
									"midpoints" : [ 616.5, 268.078125, 48.5, 268.078125 ],
									"source" : [ "obj-20", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-5", 0 ],
									"midpoints" : [ 616.5, 267.34375, 48.5, 267.34375 ],
									"source" : [ "obj-21", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-5", 0 ],
									"source" : [ "obj-23", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-17", 0 ],
									"source" : [ "obj-24", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-5", 0 ],
									"midpoints" : [ 305.5, 267.47265625, 48.5, 267.47265625 ],
									"source" : [ "obj-33", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-17", 0 ],
									"source" : [ "obj-35", 1 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-17", 0 ],
									"source" : [ "obj-35", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-5", 0 ],
									"midpoints" : [ 429.5, 268.375, 48.5, 268.375 ],
									"source" : [ "obj-38", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-5", 0 ],
									"midpoints" : [ 429.5, 268.9140625, 48.5, 268.9140625 ],
									"source" : [ "obj-39", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-5", 0 ],
									"source" : [ "obj-4", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-24", 0 ],
									"midpoints" : [ 233.0, 505.71875, 82.5, 505.71875 ],
									"source" : [ "obj-40", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-5", 0 ],
									"source" : [ "obj-43", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-16", 0 ],
									"source" : [ "obj-5", 4 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-35", 0 ],
									"source" : [ "obj-5", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-40", 0 ],
									"source" : [ "obj-5", 2 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-42", 0 ],
									"source" : [ "obj-5", 3 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-8", 0 ],
									"midpoints" : [ 140.75, 309.0, 62.5, 309.0 ],
									"source" : [ "obj-5", 1 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-9", 0 ],
									"source" : [ "obj-8", 1 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-5", 0 ],
									"midpoints" : [ 122.5, 480.453125, 25.0, 480.453125, 25.0, 268.5, 48.5, 268.5 ],
									"source" : [ "obj-9", 0 ]
								}

							}
 ]
					}
,
					"patching_rect" : [ 98.0, 152.0, 38.0, 22.0 ],
					"text" : "p text"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-2",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 0,
					"patcher" : 					{
						"fileversion" : 1,
						"appversion" : 						{
							"major" : 9,
							"minor" : 0,
							"revision" : 6,
							"architecture" : "x64",
							"modernui" : 1
						}
,
						"classnamespace" : "box",
						"rect" : [ 0.0, 26.0, 887.0, 602.0 ],
						"gridsize" : [ 15.0, 15.0 ],
						"showontab" : 1,
						"boxes" : [ 							{
								"box" : 								{
									"arrows" : 2,
									"id" : "obj-19",
									"justification" : 1,
									"maxclass" : "live.line",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 217.0, 499.0, 26.0, 13.0 ]
								}

							}
, 							{
								"box" : 								{
									"fontname" : "Lato Light",
									"id" : "obj-20",
									"linecount" : 2,
									"maxclass" : "comment",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 245.0, 488.0, 397.0, 35.0 ],
									"text" : "Draw a bezier curve with x1/x1 being the start point, cx1/cy1 cx2/cy2 the \ntwo control points and x2/y2 the end point"
								}

							}
, 							{
								"box" : 								{
									"fontname" : "Lato Light",
									"id" : "obj-21",
									"maxclass" : "comment",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 23.0, 495.0, 199.0, 21.0 ],
									"text" : "bezier x1 y1 cx1 cy1 cx2 cy2  x2 y2"
								}

							}
, 							{
								"box" : 								{
									"arrows" : 2,
									"id" : "obj-17",
									"justification" : 1,
									"maxclass" : "live.line",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 169.763992309570312, 440.0, 73.236007690429688, 12.0 ]
								}

							}
, 							{
								"box" : 								{
									"fontname" : "Lato Light",
									"id" : "obj-18",
									"linecount" : 4,
									"maxclass" : "comment",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 245.0, 414.0, 398.0, 64.0 ],
									"text" : "Draw an ellipse or elliptical arch with x/y being the center, xr the horizontal radius, ry the vertical radius\nOptional argument  start/end set the start/end angle in degrees (0º - 360º) [default 0 360]"
								}

							}
, 							{
								"box" : 								{
									"arrows" : 2,
									"id" : "obj-14",
									"justification" : 1,
									"maxclass" : "live.line",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 169.763992309570312, 355.0, 73.236007690429688, 12.0 ]
								}

							}
, 							{
								"box" : 								{
									"fontname" : "Lato Light",
									"id" : "obj-15",
									"linecount" : 3,
									"maxclass" : "comment",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 245.0, 351.0, 397.0, 50.0 ],
									"text" : "draw a circle or arc with x/y being the center, r the radius. \nOptional argument  start/end set the start/end angle in degrees (0º - 360º) [default 0 360]"
								}

							}
, 							{
								"box" : 								{
									"fontname" : "Lato Light",
									"id" : "obj-16",
									"maxclass" : "comment",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 28.0, 435.5, 137.0, 21.0 ],
									"text" : "ellipse x y rx ry start end "
								}

							}
, 							{
								"box" : 								{
									"arrows" : 2,
									"id" : "obj-10",
									"justification" : 1,
									"maxclass" : "live.line",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 169.763992309570312, 315.0, 73.236007690429688, 12.0 ]
								}

							}
, 							{
								"box" : 								{
									"fontname" : "Lato Light",
									"id" : "obj-11",
									"linecount" : 3,
									"maxclass" : "comment",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 245.0, 296.0, 397.0, 50.0 ],
									"text" : "Draw a circle or arc with x/y being the center, r the radius. \nOptional argument  start/end set the start/end angle in degrees (0º - 360º) [default 0 360]"
								}

							}
, 							{
								"box" : 								{
									"fontname" : "Lato Light",
									"id" : "obj-13",
									"maxclass" : "comment",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 28.0, 310.5, 134.0, 21.0 ],
									"text" : "circle x y r start end"
								}

							}
, 							{
								"box" : 								{
									"arrows" : 2,
									"id" : "obj-9",
									"justification" : 1,
									"maxclass" : "live.line",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 169.763992309570312, 261.0, 73.236007690429688, 12.0 ]
								}

							}
, 							{
								"box" : 								{
									"arrows" : 2,
									"id" : "obj-8",
									"justification" : 1,
									"maxclass" : "live.line",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 169.763992309570312, 222.0, 73.236007690429688, 12.0 ]
								}

							}
, 							{
								"box" : 								{
									"fontname" : "Lato Light",
									"id" : "obj-6",
									"linecount" : 2,
									"maxclass" : "comment",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 245.0, 250.0, 504.0, 35.0 ],
									"text" : "Draw a rectangle where x1/y1 is the upper left corner and x2/y2 the bottom right corner. \nOptional argument round describes the roundness of the corners (0. - 1.) [default 0.]"
								}

							}
, 							{
								"box" : 								{
									"fontname" : "Lato Light",
									"id" : "obj-7",
									"maxclass" : "comment",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 28.0, 257.0, 134.0, 21.0 ],
									"text" : "rect x1 y1 x1 x2 round"
								}

							}
, 							{
								"box" : 								{
									"fontname" : "Lato Light",
									"id" : "obj-5",
									"maxclass" : "comment",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 245.0, 218.0, 182.0, 21.0 ],
									"text" : "Draw a line from x1/y1 to x2/2"
								}

							}
, 							{
								"box" : 								{
									"fontname" : "Lato Light",
									"id" : "obj-4",
									"maxclass" : "comment",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 28.0, 218.0, 134.0, 21.0 ],
									"text" : "line x1 y1 x2 y2"
								}

							}
, 							{
								"box" : 								{
									"fontname" : "Lato Light",
									"id" : "obj-3",
									"linecount" : 2,
									"maxclass" : "comment",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 23.0, 143.0, 488.0, 35.0 ],
									"text" : "All drawing commands use a x/y coordinate system with 0/0 in the center, -1 1 in the upper left corner and 1 -1 in the bottom right corner"
								}

							}
, 							{
								"box" : 								{
									"fontname" : "Lato Light",
									"fontsize" : 24.0,
									"id" : "obj-2",
									"maxclass" : "comment",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 23.0, 92.0, 280.0, 35.0 ],
									"text" : "Drawing Commands"
								}

							}
, 							{
								"box" : 								{
									"border" : 0,
									"filename" : "helpname.js",
									"id" : "obj-12",
									"ignoreclick" : 1,
									"jsarguments" : [ "jam.ilda.compose" ],
									"maxclass" : "jsui",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"parameter_enable" : 0,
									"patching_rect" : [ 10.0, 10.0, 365.527984619140625, 57.599853515625 ]
								}

							}
 ],
						"lines" : [  ]
					}
,
					"patching_rect" : [ 55.0, 117.0, 69.0, 22.0 ],
					"text" : "p drawing"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-1",
					"maxclass" : "newobj",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 342.0, 320.0, 208.0, 22.0 ],
					"saved_object_attributes" : 					{
						"filename" : "jam.helpstarter.js",
						"parameter_enable" : 0
					}
,
					"text" : "js jam.helpstarter.js jam.ilda.compose"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-10",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 0,
					"patcher" : 					{
						"fileversion" : 1,
						"appversion" : 						{
							"major" : 9,
							"minor" : 0,
							"revision" : 6,
							"architecture" : "x64",
							"modernui" : 1
						}
,
						"classnamespace" : "box",
						"rect" : [ 84.0, 126.0, 887.0, 602.0 ],
						"default_fontsize" : 13.0,
						"gridsize" : [ 5.0, 5.0 ],
						"showontab" : 1,
						"boxes" : [ 							{
								"box" : 								{
									"hidden" : 1,
									"id" : "obj-7",
									"linecount" : 2,
									"maxclass" : "message",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 233.0, 161.0, 105.0, 38.0 ],
									"text" : ";\rmax opendoc $1"
								}

							}
, 							{
								"box" : 								{
									"border" : 0,
									"fontface" : 0,
									"fontname" : "Lato Light",
									"fontsize" : 13.0,
									"id" : "obj-33",
									"linkbold" : 1,
									"linkcolor" : [ 0.85, 0.85, 0.85, 1.0 ],
									"maxclass" : "markup",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 13.0, 124.0, 617.0, 35.0 ],
									"saved_attribute_attributes" : 									{
										"linkcolor" : 										{
											"expression" : "themecolor.live_control_fg"
										}
,
										"textcolor" : 										{
											"expression" : "themecolor.live_control_fg_off"
										}

									}
,
									"text" : "For mor information about <b>ILDA files</b>, please refer to the <a href=\"02_jam_ilda_topic.maxvig.xml\">ILDA Files and ILDA Interface</a> topic in the documentation browser.\n",
									"textcolor" : [ 0.85, 0.85, 0.85, 1.0 ]
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-28",
									"maxclass" : "message",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 404.0, 485.0, 233.0, 23.0 ],
									"text" : "visible $1, floating $1, size 330 210, $1"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-19",
									"maxclass" : "toggle",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "int" ],
									"parameter_enable" : 0,
									"patching_rect" : [ 404.0, 451.0, 24.0, 24.0 ]
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-16",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 3,
									"outlettype" : [ "jit_matrix", "bang", "" ],
									"patching_rect" : [ 404.0, 515.0, 343.0, 23.0 ],
									"text" : "jit.world ilda_compose @visible 0 @erase_color 0. 0. 0. 1."
								}

							}
, 							{
								"box" : 								{
									"bubble" : 1,
									"bubbleside" : 2,
									"fontname" : "Lato Light",
									"id" : "obj-66",
									"linecount" : 2,
									"maxclass" : "comment",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 685.0, 175.0, 84.0, 57.0 ],
									"text" : "Select frame to edit"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-65",
									"maxclass" : "number",
									"numinlets" : 1,
									"numoutlets" : 2,
									"outlettype" : [ "", "bang" ],
									"parameter_enable" : 0,
									"patching_rect" : [ 700.0, 248.0, 50.0, 23.0 ]
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-63",
									"maxclass" : "message",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 700.0, 286.0, 98.0, 23.0 ],
									"text" : "seteditframe $1"
								}

							}
, 							{
								"box" : 								{
									"bubble" : 1,
									"bubbleside" : 2,
									"fontname" : "Lato Light",
									"id" : "obj-59",
									"maxclass" : "comment",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 518.0, 239.0, 73.0, 41.0 ],
									"text" : "Save file"
								}

							}
, 							{
								"box" : 								{
									"bgcolor" : [ 0.9, 0.65, 0.05, 1.0 ],
									"fontname" : "Arial Bold",
									"hint" : "",
									"id" : "obj-60",
									"ignoreclick" : 1,
									"legacytextcolor" : 1,
									"maxclass" : "textbutton",
									"numinlets" : 1,
									"numoutlets" : 3,
									"outlettype" : [ "", "", "int" ],
									"parameter_enable" : 0,
									"patching_rect" : [ 565.5, 212.0, 20.0, 20.0 ],
									"rounded" : 60.0,
									"saved_attribute_attributes" : 									{
										"bgcolor" : 										{
											"expression" : "themecolor.lesson_step_circle"
										}

									}
,
									"text" : "4",
									"textcolor" : [ 0.34902, 0.34902, 0.34902, 1.0 ]
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-58",
									"maxclass" : "message",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 532.0, 286.0, 45.0, 23.0 ],
									"text" : "export"
								}

							}
, 							{
								"box" : 								{
									"bubble" : 1,
									"bubbleside" : 2,
									"fontname" : "Lato Light",
									"id" : "obj-56",
									"linecount" : 2,
									"maxclass" : "comment",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 612.0, 219.0, 80.0, 57.0 ],
									"text" : "Remove all frames"
								}

							}
, 							{
								"box" : 								{
									"bubble" : 1,
									"bubbleside" : 2,
									"fontname" : "Lato Light",
									"id" : "obj-54",
									"linecount" : 2,
									"maxclass" : "comment",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 404.0, 226.0, 100.0, 57.0 ],
									"text" : "Add a new frame"
								}

							}
, 							{
								"box" : 								{
									"bubble" : 1,
									"bubbleside" : 2,
									"fontname" : "Lato Light",
									"id" : "obj-50",
									"linecount" : 2,
									"maxclass" : "comment",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 267.0, 219.0, 78.0, 57.0 ],
									"text" : "Draw some shapes"
								}

							}
, 							{
								"box" : 								{
									"bgcolor" : [ 0.9, 0.65, 0.05, 1.0 ],
									"fontname" : "Arial Bold",
									"hint" : "",
									"id" : "obj-51",
									"ignoreclick" : 1,
									"legacytextcolor" : 1,
									"maxclass" : "textbutton",
									"numinlets" : 1,
									"numoutlets" : 3,
									"outlettype" : [ "", "", "int" ],
									"parameter_enable" : 0,
									"patching_rect" : [ 325.0, 193.5, 20.0, 20.0 ],
									"rounded" : 60.0,
									"saved_attribute_attributes" : 									{
										"bgcolor" : 										{
											"expression" : "themecolor.lesson_step_circle"
										}

									}
,
									"text" : "3",
									"textcolor" : [ 0.34902, 0.34902, 0.34902, 1.0 ]
								}

							}
, 							{
								"box" : 								{
									"bubble" : 1,
									"bubblepoint" : 0.01,
									"bubbleside" : 2,
									"fontname" : "Lato Light",
									"id" : "obj-49",
									"linecount" : 2,
									"maxclass" : "comment",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 81.0, 178.0, 102.0, 57.0 ],
									"text" : "Set the drawing color"
								}

							}
, 							{
								"box" : 								{
									"bgcolor" : [ 0.9, 0.65, 0.05, 1.0 ],
									"fontname" : "Arial Bold",
									"hint" : "",
									"id" : "obj-47",
									"ignoreclick" : 1,
									"legacytextcolor" : 1,
									"maxclass" : "textbutton",
									"numinlets" : 1,
									"numoutlets" : 3,
									"outlettype" : [ "", "", "int" ],
									"parameter_enable" : 0,
									"patching_rect" : [ 163.0, 153.0, 20.0, 20.0 ],
									"rounded" : 60.0,
									"saved_attribute_attributes" : 									{
										"bgcolor" : 										{
											"expression" : "themecolor.lesson_step_circle"
										}

									}
,
									"text" : "2",
									"textcolor" : [ 0.34902, 0.34902, 0.34902, 1.0 ]
								}

							}
, 							{
								"box" : 								{
									"bubble" : 1,
									"bubblepoint" : 0.01,
									"bubbleside" : 2,
									"fontname" : "Lato Light",
									"id" : "obj-46",
									"linecount" : 2,
									"maxclass" : "comment",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 412.0, 388.0, 102.0, 57.0 ],
									"text" : "Start the render context"
								}

							}
, 							{
								"box" : 								{
									"bgcolor" : [ 0.9, 0.65, 0.05, 1.0 ],
									"fontname" : "Arial Bold",
									"hint" : "",
									"id" : "obj-93",
									"ignoreclick" : 1,
									"legacytextcolor" : 1,
									"maxclass" : "textbutton",
									"numinlets" : 1,
									"numoutlets" : 3,
									"outlettype" : [ "", "", "int" ],
									"parameter_enable" : 0,
									"patching_rect" : [ 494.0, 362.0, 20.0, 20.0 ],
									"rounded" : 60.0,
									"saved_attribute_attributes" : 									{
										"bgcolor" : 										{
											"expression" : "themecolor.lesson_step_circle"
										}

									}
,
									"text" : "1",
									"textcolor" : [ 0.34902, 0.34902, 0.34902, 1.0 ]
								}

							}
, 							{
								"box" : 								{
									"bubble" : 1,
									"bubbleside" : 0,
									"fontname" : "Lato Light",
									"id" : "obj-45",
									"linecount" : 2,
									"maxclass" : "comment",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 151.0, 421.0, 100.0, 57.0 ],
									"text" : "Frame selected for editing"
								}

							}
, 							{
								"box" : 								{
									"bubble" : 1,
									"fontname" : "Lato Light",
									"id" : "obj-44",
									"linecount" : 2,
									"maxclass" : "comment",
									"numinlets" : 1,
									"numoutlets" : 0,
									"patching_rect" : [ 301.0, 372.0, 102.0, 42.0 ],
									"text" : "Total number of frames"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-42",
									"maxclass" : "number",
									"numinlets" : 1,
									"numoutlets" : 2,
									"outlettype" : [ "", "bang" ],
									"parameter_enable" : 0,
									"patching_rect" : [ 241.0, 382.0, 50.0, 23.0 ]
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-40",
									"maxclass" : "number",
									"numinlets" : 1,
									"numoutlets" : 2,
									"outlettype" : [ "", "bang" ],
									"parameter_enable" : 0,
									"patching_rect" : [ 176.0, 382.0, 50.0, 23.0 ]
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-38",
									"maxclass" : "message",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 412.0, 286.0, 85.0, 23.0 ],
									"text" : "appendframe"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-35",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 2,
									"outlettype" : [ "bang", "" ],
									"patching_rect" : [ 47.0, 468.0, 29.5, 23.0 ],
									"text" : "t b l"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-31",
									"maxclass" : "message",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 643.0, 286.0, 37.0, 23.0 ],
									"text" : "clear"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-29",
									"maxclass" : "message",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 306.0, 286.0, 89.0, 23.0 ],
									"text" : "circle 0. 0. 0.9"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-26",
									"maxclass" : "button",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "bang" ],
									"parameter_enable" : 0,
									"patching_rect" : [ 13.0, 257.0, 24.0, 24.0 ]
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-24",
									"maxclass" : "message",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 81.0, 468.0, 60.0, 23.0 ],
									"text" : "frame $1"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-17",
									"maxclass" : "newobj",
									"numinlets" : 2,
									"numoutlets" : 2,
									"outlettype" : [ "dictionary", "" ],
									"patching_rect" : [ 47.0, 515.0, 261.0, 23.0 ],
									"text" : "jam.jit.gl.ilda.sketch @drawto ilda_compose"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-14",
									"maxclass" : "message",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 176.0, 286.0, 126.0, 23.0 ],
									"text" : "rect -0.9 0.9 0.9 -0.9"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-13",
									"maxclass" : "swatch",
									"numinlets" : 3,
									"numoutlets" : 2,
									"outlettype" : [ "", "float" ],
									"parameter_enable" : 0,
									"patching_rect" : [ 47.0, 239.0, 128.0, 32.0 ],
									"saturation" : 1.0
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-12",
									"maxclass" : "message",
									"numinlets" : 2,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"patching_rect" : [ 47.0, 286.0, 119.0, 23.0 ],
									"text" : "drawcolor $1 $2 $3"
								}

							}
, 							{
								"box" : 								{
									"id" : "obj-5",
									"maxclass" : "newobj",
									"numinlets" : 1,
									"numoutlets" : 5,
									"outlettype" : [ "", "", "int", "int", "list" ],
									"patching_rect" : [ 47.0, 331.0, 278.0, 23.0 ],
									"text" : "jam.ilda.compose"
								}

							}
, 							{
								"box" : 								{
									"border" : 0,
									"filename" : "jam.helpdetails.js",
									"id" : "obj-2",
									"ignoreclick" : 1,
									"jsarguments" : [ "jam.ilda.compose" ],
									"maxclass" : "jsui",
									"numinlets" : 1,
									"numoutlets" : 1,
									"outlettype" : [ "" ],
									"parameter_enable" : 0,
									"patching_rect" : [ 10.0, 10.0, 620.0, 125.0 ]
								}

							}
 ],
						"lines" : [ 							{
								"patchline" : 								{
									"destination" : [ "obj-5", 0 ],
									"midpoints" : [ 56.5, 320.0, 56.5, 320.0 ],
									"source" : [ "obj-12", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-12", 0 ],
									"source" : [ "obj-13", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-5", 0 ],
									"midpoints" : [ 185.5, 320.0, 56.5, 320.0 ],
									"source" : [ "obj-14", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-28", 0 ],
									"source" : [ "obj-19", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-17", 0 ],
									"source" : [ "obj-24", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-5", 0 ],
									"midpoints" : [ 22.5, 319.6875, 56.5, 319.6875 ],
									"source" : [ "obj-26", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-16", 0 ],
									"source" : [ "obj-28", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-5", 0 ],
									"midpoints" : [ 315.5, 320.0, 56.5, 320.0 ],
									"source" : [ "obj-29", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-5", 0 ],
									"midpoints" : [ 652.5, 320.0, 56.5, 320.0 ],
									"source" : [ "obj-31", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-7", 0 ],
									"hidden" : 1,
									"source" : [ "obj-33", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-17", 0 ],
									"source" : [ "obj-35", 1 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-17", 0 ],
									"source" : [ "obj-35", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-5", 0 ],
									"midpoints" : [ 421.5, 320.0, 56.5, 320.0 ],
									"source" : [ "obj-38", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-24", 0 ],
									"midpoints" : [ 185.5, 418.33984375, 90.5, 418.33984375 ],
									"source" : [ "obj-40", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-35", 0 ],
									"source" : [ "obj-5", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-40", 0 ],
									"source" : [ "obj-5", 2 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-42", 0 ],
									"source" : [ "obj-5", 3 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-5", 0 ],
									"midpoints" : [ 541.5, 320.0, 56.5, 320.0 ],
									"source" : [ "obj-58", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-5", 0 ],
									"midpoints" : [ 709.5, 320.0, 56.5, 320.0 ],
									"source" : [ "obj-63", 0 ]
								}

							}
, 							{
								"patchline" : 								{
									"destination" : [ "obj-63", 0 ],
									"source" : [ "obj-65", 0 ]
								}

							}
 ]
					}
,
					"patching_rect" : [ 29.0, 83.0, 50.0, 22.0 ],
					"saved_object_attributes" : 					{
						"fontsize" : 13.0
					}
,
					"text" : "p basic",
					"varname" : "basic_tab"
				}

			}
, 			{
				"box" : 				{
					"border" : 0,
					"filename" : "helpname.js",
					"id" : "obj-12",
					"ignoreclick" : 1,
					"jsarguments" : [ "jam.ilda.compose" ],
					"maxclass" : "jsui",
					"numinlets" : 1,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"parameter_enable" : 0,
					"patching_rect" : [ 10.0, 10.0, 365.527984619140625, 57.599853515625 ]
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-13",
					"maxclass" : "newobj",
					"numinlets" : 0,
					"numoutlets" : 0,
					"patcher" : 					{
						"fileversion" : 1,
						"appversion" : 						{
							"major" : 9,
							"minor" : 0,
							"revision" : 6,
							"architecture" : "x64",
							"modernui" : 1
						}
,
						"classnamespace" : "box",
						"rect" : [ 0.0, 26.0, 887.0, 602.0 ],
						"gridsize" : [ 15.0, 15.0 ],
						"showontab" : 1,
						"boxes" : [  ],
						"lines" : [  ]
					}
,
					"patching_rect" : [ 312.0, 285.0, 50.0, 22.0 ],
					"text" : "p ?",
					"varname" : "q_tab"
				}

			}
 ],
		"lines" : [  ],
		"dependency_cache" : [ 			{
				"name" : "helpname.js",
				"bootpath" : "C74:/help/resources",
				"type" : "TEXT",
				"implicit" : 1
			}
, 			{
				"name" : "jam.helpdetails.js",
				"bootpath" : "~/Documents/Workspace/Xcode/jam/javascript",
				"patcherrelativepath" : "../javascript",
				"type" : "TEXT",
				"implicit" : 1
			}
, 			{
				"name" : "jam.helpstarter.js",
				"bootpath" : "~/Documents/Workspace/Xcode/jam/javascript",
				"patcherrelativepath" : "../javascript",
				"type" : "TEXT",
				"implicit" : 1
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
		"autosave" : 0
	}

}

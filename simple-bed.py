"""
Simple-Bed
name: simple-bed.py
by:   Paul8043
date: 2023-05-14
desc:
    This python module builds a simple-bed.
license:
    Copyright 2023 Paul8043
    Licensed under the MIT License (see LICENSE-file)
"""

from collections import namedtuple
from build123d import *
from ocp_vscode import show, show_object, reset_show, set_port, set_defaults, get_defaults

set_port(3939)

DrillHole = namedtuple("DrillHole", "diameter depth")

# all measures are in mm

measures ={
    "@.thickness.1":18,
    "@.thickness.2":27,
    "@.side.length":120,
    "@.bearing":60,
    "@.margin":10,
    "@.overhang":5,
    "@.rib.jut":16,
    "@.diameter.1":6+1.5,   # M6
    "@.diameter.2":10,      # jig-corner
    "@.diameter.3":10.5,    # pilot hole
    "mattress.height":180,
    "mattress.width":900,
    "mattress.length":2100,
    "mattress.altitude":465,
    "storage.height":260,
    "storage.width":900,
    "storage.length":1310,
    "rampa.m6":DrillHole(diameter=10.5,depth=18+1),
}

# derived measures

measures["@.width.1"]           = measures["@.side.length"]*0.5
measures["@.width.2"]           = measures["@.side.length"]*2/3
measures["@.width.3"]           = measures["@.side.length"]

measures["stringer.thickness"] = measures["@.thickness.2"]
measures["stringer.width"]     = measures["@.width.3"]
measures["stringer.length"]    = measures["mattress.length"]*0.5

measures["ledger.thickness"]   = measures["@.thickness.2"]
measures["ledger.width"]       = measures["@.width.3"]
measures["ledger.length"]      = measures["mattress.width"]-2*measures["@.width.3"]

measures["rib.thickness"]      = measures["@.thickness.2"]
measures["rib.width"]          = measures["@.width.2"]
measures["rib.length.1"]       = measures["ledger.length"]+2*(measures["@.thickness.2"]+measures["@.rib.jut"])
measures["rib.length.2"]       = measures["stringer.length"]-1.5*measures["@.side.length"]+2*(measures["@.thickness.2"]+measures["@.rib.jut"])
measures["rib.cut.width"]      = measures["@.thickness.2"]
measures["rib.cut.depth"]      = 4.5
measures["rib.cut.air"]        = 0

measures["batten.thickness"]   = measures["@.thickness.1"]
measures["batten.width.1"]     = measures["@.width.1"]
measures["batten.width.3"]     = measures["@.width.3"]
measures["batten.length"]      = measures["mattress.width"]
measures["batten.gap"]         = 78
measures["batten.extra"]       = (measures["stringer.length"]-measures["batten.width.3"]
-4*measures["batten.width.3"]-5*measures["batten.gap"]-measures["batten.width.1"])/2

measures["jamb.thickness"]     = measures["@.thickness.2"]      # -2.5  for 3D-printing
measures["jamb.side"]          = measures["@.side.length"]
measures["jamb.length"]        = measures["mattress.altitude"]-measures["@.thickness.1"]-measures["@.thickness.2"]
measures["jamb.cut.width"]     = measures["@.thickness.1"]
measures["jamb.cut.depth"]     = measures["@.width.2"]
measures["jamb.cut.air"]       = 0                             # 1.3  for 3D-printing

# models

def build_stringer(name="stringer",length=None,width=None,thickness=None):
    m2d = Sketch()+Rectangle(length,width)
    m3d = Part()+extrude(m2d,0.5*thickness,both=True)
    #show_object(m2d,name="m2d_"+name,options={"alpha":0.2,"color":(255,0,0)})
    #show_object(m3d,name="m3d_"+name,options={"alpha":0.2,"color":(255,170,0)})
    return m3d

def build_ledger(name="leger",length=None,width=None,thickness=None):
    m2d = Sketch()+Rectangle(length,width)
    m3d = Part()+extrude(m2d,0.5*thickness,both=True)
    #show_object(m2d,name="m2d_"+name,options={"alpha":0.2,"color":(255,0,0)})
    #show_object(m3d,name="m3d_"+name,options={"alpha":0.2,"color":(255,170,0)})
    return m3d   

def build_batten(name="batten",length=None,width=None,thickness=None):
    m2d = Sketch()+Rectangle(length,width)
    m3d = Part()+extrude(m2d,0.5*thickness,both=True)
    #show_object(m2d,name="m2d_"+name,options={"alpha":0.2,"color":(255,0,0)})
    #show_object(m3d,name="m3d_"+name,options={"alpha":0.2,"color":(255,170,0)})
    return m3d 

def build_jamb_side(
        name="jamb_side",
        margin=None,
        overhang=None,
        length=None,
        width=None,
        thickness=None,
        cut_width=None,
        cut_depth=None,
        cut_air=None
    ):
    pts = [
        (0,0),                                              # P0   unused
        (+0.5*width,0),                                     # P1   raw plate
        (+0.5*width,length),                                # P2   raw plate
        (-0.5*width,length),                                # P3   raw plate
        (-0.5*width,0),                                     # P4   raw plate
        (-0.5*cut_width-cut_air,length-cut_depth-cut_air),  # P5   cut-off
        (-0.5*cut_width-cut_air,length+overhang+cut_air),   # P6   cut-off
        (+0.5*cut_width+cut_air,length+overhang+cut_air),   # P7   cut-off
        (+0.5*cut_width+cut_air,length-cut_depth-cut_air),  # P8   cut-off
        (+0.5*width+margin,0-margin),                       # P9   border
        (+0.5*width+margin,length+margin),                  # P10  border
        (-0.5*width-margin,length+margin),                  # P11  border
        (-0.5*width-margin,0-margin),                       # P12  border
    ]
    border_pts = [pts[9],pts[10],pts[11],pts[12]]
    plate_pts  = [pts[1],pts[2],pts[3],pts[4]]
    cut_pts    = [pts[5],pts[6],pts[7],pts[8]]
    border_pl  = Polyline(*border_pts,close=True)
    plate_pl   = Polyline(*plate_pts,close=True)
    cut_pl     = Polyline(*cut_pts,close=True)
    m1d        = Curve()+[border_pl,plate_pl,cut_pl]       
    border_fc  = make_face(Plane.XY*border_pl)
    plate_fc   = make_face(Plane.XY*plate_pl)
    cut_fc     = make_face(Plane.XY*cut_pl)
    m2d_full   = Sketch()+plate_fc
    m2d_cut    = Sketch()+plate_fc-cut_fc
    m3d_full   = Part()+extrude(m2d_full,-0.5*thickness,both=True)  
    m3d_cut    = Part()+extrude(m2d_cut,-0.5*thickness,both=True)
    m1d.export_svg("m1d_"+name+".svg",(0,0.5*length,1E6))
    #show_object(m1d,name="m1d_"+name,options={"alpha":0.2,"color":(25,25,25)})
    #show_object(m2d_full,name="m2d_full_"+name,options={"alpha":0.2,"color":(170,255,0)})
    #show_object(m2d_cut,name="m2d_cut_"+name,options={"alpha":0.2,"color":(170,255,0)})
    #show_object(m3d_full,name="m3d_full_"+name,options={"alpha":0.2,"color":(255,170,0)})
    #show_object(m3d_cut,name="m3d_cut_"+name,options={"alpha":0.2,"color":(255,170,0)})
    return m3d_full,m3d_cut

def build_rib(
        name="rib",
        margin=None,
        overhang=None,
        length=None,
        width=None,
        jut=None,
        thickness=None,
        cut_width=None,
        cut_depth=None,
        cut_air=None
    ):
    x2 = 0.5*length-jut
    x1 = x2-cut_width
    pts = [
        (0,0),                                      # P0   unused
        (+0.5*length,-0.5*width),                   # P1   raw plate
        (+0.5*length,+0.5*width),                   # P2   raw plate
        (-0.5*length,+0.5*width),                   # P3   raw plate
        (-0.5*length,-0.5*width),                   # P4   raw plate

        (-x2-cut_air,-0.5*width-overhang-cut_air),  # P5   cut-off left
        (-x2-cut_air,+0.5*width+overhang+cut_air),  # P6   cut-off left
        (-x1+cut_air,+0.5*width+overhang+cut_air),  # P7   cut-off left
        (-x1+cut_air,-0.5*width-overhang-cut_air),  # P8   cut-off left

        (+x1-cut_air,-0.5*width-overhang-cut_air),  # P9   cut-off right
        (+x1-cut_air,+0.5*width+overhang+cut_air),  # P10  cut-off right
        (+x2+cut_air,+0.5*width+overhang+cut_air),  # P11  cut-off right
        (+x2+cut_air,-0.5*width-overhang-cut_air),  # P12  cut-off right

        (+0.5*length+margin,-0.5*width-margin),     # P13  border
        (+0.5*length+margin,+0.5*width+margin),     # P14  border
        (-0.5*length-margin,+0.5*width+margin),     # P15  border
        (-0.5*length-margin,-0.5*width-margin),     # P16  border
    ]
    border_pts    = [pts[13],pts[14],pts[15],pts[16]]
    plate_pts     = [pts[1],pts[2],pts[3],pts[4]]
    cut_left_pts  = [pts[5],pts[6],pts[7],pts[8]]
    cut_right_pts = [pts[9],pts[10],pts[11],pts[12]]
    border_pl     = Polyline(*border_pts,close=True)
    plate_pl      = Polyline(*plate_pts,close=True)
    cut_left_pl   = Polyline(*cut_left_pts,close=True)
    cut_right_pl  = Polyline(*cut_right_pts,close=True)
    m1d           = Curve()+[border_pl,plate_pl,cut_left_pl,cut_right_pl]       
    border_fc     = make_face(Plane.XY*border_pl)
    plate_fc      = make_face(Plane.XY*plate_pl)
    cut_left_fc   = make_face(Plane.XY*cut_left_pl)
    cut_right_fc  = make_face(Plane.XY*cut_right_pl)
    m3d_full      = Part()+extrude(plate_fc,-0.5*thickness,both=True)
    m3d_cut_left  = Part()+extrude(cut_left_fc,-cut_depth-cut_air,both=True)
    m3d_cut_right = Part()+extrude(cut_right_fc,-cut_depth-cut_air,both=True)
    m3d_cut       = m3d_full
    m3d_cut       -= Pos(0,0,+0.5*thickness)*m3d_cut_left
    m3d_cut       -= Pos(0,0,+0.5*thickness)*m3d_cut_right
    m3d_cut       -= Pos(0,0,-0.5*thickness)*m3d_cut_left
    m3d_cut       -= Pos(0,0,-0.5*thickness)*m3d_cut_right
    m1d.export_svg("m1d_"+name+".svg",(0,0.5*length,1E6))
    #show_object(m1d,name="m1d_"+name,options={"alpha":0.2,"color":(25,25,25)})
    #show_object(m3d_full,name="m3d_full_"+name,options={"alpha":0.2,"color":(255,170,0)})
    #show_object(m3d_cut,name="m3d_cut_"+name,options={"alpha":0.2,"color":(255,170,0)})
    return m3d_cut

def build_simple_bed(args):
    stringer = build_stringer(
        name="stringer",
        length=args["stringer.length"],
        width=args["stringer.width"],    
        thickness=args["stringer.thickness"],
    )
    ledger = build_ledger(
        name="ledger",
        length=args["ledger.length"],
        width=args["ledger.width"],
        thickness=args["ledger.thickness"],        
    )
    batten_narrow = build_batten(
        name="batten_narrow",
        length=args["batten.length"],
        width=args["batten.width.1"],
        thickness=args["batten.thickness"],        
    )
    batten_broad = build_batten(
        name="batten_broad",
        length=args["batten.length"],
        width=args["batten.width.3"],
        thickness=args["batten.thickness"],        
    )
    jamb_side_narrow_full, jamb_side_narrow_cut = build_jamb_side(
        name="jamb_side_narrow",
        margin=args["@.margin"],
        overhang=args["@.overhang"],
        length=args["jamb.length"],
        width=args["jamb.side"]-2*args["jamb.thickness"],
        thickness=args["jamb.thickness"],
        cut_width=args["jamb.cut.width"],
        cut_depth=args["jamb.cut.depth"],
        cut_air=args["jamb.cut.air"]
    )
    jamb_side_broad_full, jamb_side_broad_cut = build_jamb_side(
        name="jamb_side_broad",
        margin=args["@.margin"],
        overhang=args["@.overhang"],
        length=args["jamb.length"],
        width=args["jamb.side"],
        thickness=args["jamb.thickness"],
        cut_width=args["jamb.cut.width"],
        cut_depth=args["jamb.cut.depth"],
        cut_air=args["jamb.cut.air"]
    )
    rib_short_cut = build_rib(
        name="rib_short",
        margin=args["@.margin"],
        overhang=args["@.overhang"],
        length=args["rib.length.1"],
        width=args["rib.width"],
        jut=args["@.rib.jut"],
        thickness=args["rib.thickness"],
        cut_width=args["rib.cut.width"],
        cut_depth=args["rib.cut.depth"],
        cut_air=args["rib.cut.air"]
    )
    rib_long_cut = build_rib(
        name="rib_long",
        margin=args["@.margin"],
        overhang=args["@.overhang"],
        length=args["rib.length.2"],
        width=args["rib.width"],
        jut=args["@.rib.jut"],
        thickness=args["rib.thickness"],
        cut_width=args["rib.cut.width"],
        cut_depth=args["rib.cut.depth"],
        cut_air=args["rib.cut.air"]
    )
    bt  = args["batten.thickness"]
    bw1 = args["batten.width.1"]
    bw3 = args["batten.width.3"]
    be  = args["batten.extra"]
    bg  = args["batten.gap"]
    js  = args["jamb.side"]
    jt  = args["jamb.thickness"]
    jl  = args["jamb.length"]
    ll  = args["ledger.length"]
    lt  = args["ledger.thickness"]
    st  = args["stringer.thickness"]
    sl  = args["stringer.length"]
    rw  = args["rib.width"]
    rl  = args["rib.length.2"]
    rj  = args["@.rib.jut"]
    jamb_corner = Part()+[
        Pos(0,-0.5*(js-jt),0)*Rot(+90,0,0)*jamb_side_broad_full,
        Pos(0,+0.5*(js-jt),0)*Rot(+90,0,0)*jamb_side_broad_cut,
        Pos(-0.5*(js-jt),0,0)*Rot(0,0,+90)*Rot(+90,0,0)*jamb_side_narrow_full,
        Pos(+0.5*(js-jt),0,0)*Rot(0,0,+90)*Rot(+90,0,0)*jamb_side_narrow_cut,
    ]
    jamb_FL = Rot(0,0,0)*jamb_corner
    jamb_BL = Rot(0,0,-90)*jamb_corner
    jamb_BR = Rot(0,0,-180)*jamb_corner
    jamb_FR = Rot(0,0,-270)*jamb_corner
    #show_object(jamb_FL,name="jamb_FL",options={"alpha":0.2,"color":(255,170,0)})
    #show_object(jamb_BL,name="jamb_BL",options={"alpha":0.2,"color":(255,170,0)})
    #show_object(jamb_BR,name="jamb_BR",options={"alpha":0.2,"color":(255,170,0)})
    #show_object(jamb_FR,name="jamb_FR",options={"alpha":0.2,"color":(255,170,0)})
    jamb_middle = Part()+[
        Pos(0,-0.5*(js-jt),0)*Rot(+90,0,0)*jamb_side_broad_full,
        Pos(0,+0.5*(js-jt),0)*Rot(+90,0,0)*jamb_side_broad_cut,
        Pos(-0.5*(js-jt),0,0)*Rot(0,0,+90)*Rot(+90,0,0)*jamb_side_narrow_cut,
        Pos(+0.5*(js-jt),0,0)*Rot(0,0,+90)*Rot(+90,0,0)*jamb_side_narrow_cut,
    ]
    jamb_FM = Rot(0,0,0)*jamb_middle
    jamb_BM = Rot(0,0,-180)*jamb_middle
    #show_object(jamb_FM,name="jamb_FM",options={"alpha":0.2,"color":(255,170,0)})
    #show_object(jamb_BM,name="jamb_BM",options={"alpha":0.2,"color":(255,170,0)})
    rib_short = Rot(+90,0,0)*rib_short_cut
    rib_short = Rot(0,0,+90)*rib_short
    rib_short = Pos(0,0,jl-0.5*rw)*rib_short
    #show_object(rib_short,name="rib_short",options={"alpha":0.2,"color":(255,170,0)})
    rib_long  = Rot(+90,0,0)*rib_long_cut
    rib_long  = Pos(0,0,jl-0.5*rw)*rib_long
    #show_object(rib_long,name="rib_long",options={"alpha":0.2,"color":(255,170,0)})
    rxo = 0.5*rl+0.5*js-jt-rj
    frame = Part()+[
        Pos(0,-0.5*(ll+js),0)*jamb_FM,
        Pos(0,+0.5*(ll+js),0)*jamb_BM,
        Pos(-(sl-0.5*js),-0.5*(ll+js),0)*jamb_FL,
        Pos(-(sl-0.5*js),+0.5*(ll+js),0)*jamb_BL,
        Pos(+(sl-0.5*js),-0.5*(ll+js),0)*jamb_FR,
        Pos(+(sl-0.5*js),+0.5*(ll+js),0)*jamb_BR,
        Pos(-(sl-0.5*js),0,0)*rib_short,           # left
        Pos(0,0,0)*rib_short,                      # middle
        Pos(+(sl-0.5*js),0,0)*rib_short,           # right
        Pos(-rxo,+0.5*(ll+js),0)*rib_long,         # back left
        Pos(-rxo,-0.5*(ll+js),0)*rib_long,         # front left
        Pos(+rxo,+0.5*(ll+js),0)*rib_long,         # back right
        Pos(+rxo,-0.5*(ll+js),0)*rib_long,         # front right
    ]
    #show_object(frame,name="frame",options={"alpha":1.0,"color":(255,170,0)})
    batten_half = Pos(0,0,0.5*(st+bt))*Rot(0,0,+90)*batten_narrow
    batten_full = Pos(0,0,0.5*(st+bt))*Rot(0,0,+90)*batten_broad
    xo = -0.5*sl+bw3+be+bg+0.5*bw3
    duckboard = Part()+[
        Pos(0,-0.5*(ll+js),0)*stringer,
        Pos(0,+0.5*(ll+js),0)*stringer,
        #Pos(-0.5*(sl-bw3),0,0)*batten_full,
        Pos(xo+0*(bg+bw3),0,0)*batten_full,
        Pos(xo+1*(bg+bw3),0,0)*batten_full,
        Pos(xo+2*(bg+bw3),0,0)*batten_full,
        Pos(xo+3*(bg+bw3),0,0)*batten_full,
        #Pos(+0.5*(sl-bw1),0,0)*batten_half,  
    ]
    #show_object(duckboard,name="duckboard",options={"alpha":1.0,"color":(255,170,0)})
    joist = Part()+[
        Rot(0,0,+90)*ledger,
        batten_full,
    ]
    #show_object(joist,name="joist",options={"alpha":1.0,"color":(255,170,0)})
    duckboard_left  = Rot(0,0,0)*duckboard
    duckboard_right = Rot(0,0,+180)*duckboard
    bed = frame+[
        Pos(-0.5*sl,0,jl+0.5*st+1)*duckboard_left,
        Pos(+0.5*sl,0,jl+0.5*st+1)*duckboard_right,
        Pos(-sl+0.5*js,0,jl+0.5*lt+1)*joist,         # left
        Pos(0,0,jl+0.5*lt+1)*joist,                  # middle
        Pos(+sl-0.5*js,0,jl+0.5*lt+1)*joist,         # right
    ]
    #show_object(bed,name="bed",options={"alpha":1.0,"color":(255,170,0)})
    return bed

def build_jig(name,measures):
    m=measures["@.margin"]
    b=measures["@.bearing"]
    js=measures["jamb.side"]
    jt=measures["jamb.thickness"]
    d2=measures["@.diameter.2"]
    d3=measures["@.diameter.3"]
    pts = [
        (0,0),                          # P0   unused
        (+(0.5*js+b+m),-(0.5*js+b+m)),  # P1   border
        (+(0.5*js+b+m),+(0.5*js+b+m)),  # P2   border
        (-(0.5*js+b+m),+(0.5*js+b+m)),  # P3   border
        (-(0.5*js+b+m),-(0.5*js+b+m)),  # P4   border
        (+(0.5*js+b),-(0.5*js+b)),      # P5   bearing
        (+(0.5*js+b),+(0.5*js+b)),      # P6   bearing
        (-(0.5*js+b),+(0.5*js+b)),      # P7   bearing
        (-(0.5*js+b),-(0.5*js+b)),      # P8   bearing
        (-(0.5*js),-(0.5*js)),          # P9   jamb outside
        (-(0.5*js),+(0.5*js)),          # P10  jamb outside
        (+(0.5*js),+(0.5*js)),          # P11  jamb outside
        (+(0.5*js),-(0.5*js)),          # P12  jamb outside
        (+(0.5*js-jt),-(0.5*js-jt)),    # P13  jamb inside
        (+(0.5*js-jt),+(0.5*js-jt)),    # P14  jamb inside
        (-(0.5*js-jt),+(0.5*js-jt)),    # P15  jamb inside
        (-(0.5*js-jt),-(0.5*js-jt)),    # P16  jamb inside
        (+0.25*js,-0.5*(js-jt)),        # P17  hole
        (+0.25*js,+0.5*(js-jt)),        # P18  hole
        (-0.25*js,+0.5*(js-jt)),        # P19  hole
        (-0.25*js,-0.5*(js-jt)),        # P20  hole
    ]
    p1  = [pts[1],pts[2],pts[3],pts[4]]
    l1  = Polyline(*p1,close=True)
    p2  = [pts[5],pts[6],pts[7],pts[8]]
    l2  = Polyline(*p2,close=True)
    a9  = CenterArc(pts[9],0.5*d2,90,270)
    a10 = CenterArc(pts[10],0.5*d2,0,270)
    a11 = CenterArc(pts[11],0.5*d2,270,270)
    a12 = CenterArc(pts[12],0.5*d2,180,270)
    l3  = a9+Line(a9@0,a10@1)+a10+Line(a10@0,a11@1)+a11+Line(a11@0,a12@1)+a12+Line(a12@0,a9@1)
    p4  = [pts[13],pts[14],pts[15],pts[16]]
    l4  = Polyline(*p4,close=True)
    c17 =CenterArc(pts[17],0.5*d3,0,360)
    c18 =CenterArc(pts[18],0.5*d3,0,360)
    c19 =CenterArc(pts[19],0.5*d3,0,360)
    c20 =CenterArc(pts[20],0.5*d3,0,360)
    m1d = Curve()+[l1,l2,l3,l4,c17,c18,c19,c20]
    m1d.export_svg("m1d_"+name+".svg",(0,0,1E6))
    #show_object(m1d,name="m1d_"+name,options={"alpha":0.2,"color":(170,255,170)})
    return m1d

def dump(args) -> None:
    print("")
    for key in sorted(args):
        print(f"{key}:{args[key]}")
    print("")
    return

# main

if __name__ == "__main__":
    dump(measures)
    simple_bed = build_simple_bed(measures)
    show_object(simple_bed,name="simple_bed",options={"alpha":1.0,"color":(255,170,0)})
    jig = build_jig("jig",measures)
    #show_object(jig,name="jig",options={"alpha":1.0,"color":(170,255,170)})
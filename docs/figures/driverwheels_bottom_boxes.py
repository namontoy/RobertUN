# Annotated bottom photo: docs/images/prototype/Prototype_DriverWheels_Bottom.jpg -> docs/images/prototype/Prototype_DriverWheels_Bottom_boxes.jpg
# Run driverwheels_bottom_mask.py first if the mask is missing.
# The bottom view is mirrored left-right with respect to the top photo.
import os
from PIL import Image, ImageDraw, ImageFont
HERE=os.path.dirname(os.path.abspath(__file__))
SRC=HERE+'/../images/prototype/Prototype_DriverWheels_Bottom.jpg'
DST=HERE+'/../images/prototype/Prototype_DriverWheels_Bottom_boxes.jpg'
im=Image.open(SRC).convert('RGB')
mask=Image.open(HERE+'/driverwheels_bottom_mask.png').convert('L')
im=Image.composite(im,Image.new('RGB',im.size,(128,128,128)),mask)
OX,PADR=0,0
cv=Image.new('RGB',(im.width+OX+PADR,im.height),(128,128,128)); cv.paste(im,(OX,0)); im=cv
d=ImageDraw.Draw(im)
k=1512/1125
F='/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'
fb=ImageFont.truetype(F,34); fl=ImageFont.truetype(F,27)
# (n, label, display-coords box, colour, badge corner)
E=[
 (1,'Battery input terminal (back of top box 3)', (78,148,252,228),   '#FF2D2D','tr'),
 (2,'Buck converter, 12 V out (LM2596-type)',      (230,362,558,1032), '#00B4FF','tr'),
 (3,'Buck converter, 3V3 out (LM2596-type)',      (583,302,914,987),  '#7CFF00','br'),
]
R=27
def ink(c):
    r,g,b=(int(c[i:i+2],16) for i in (1,3,5))
    return 'black' if 0.299*r+0.587*g+0.114*b>140 else 'white'
for n,lab,b,c,corner in E:
    x0,y0,x1,y1=[v*k for v in b]; x0+=OX; x1+=OX
    d.rectangle((x0,y0,x1,y1),outline='black',width=10)
    d.rectangle((x0,y0,x1,y1),outline=c,width=6)
    cx=x0 if 'l' in corner else x1; cy=y0 if 't' in corner else y1
    d.ellipse((cx-R,cy-R,cx+R,cy+R),fill=c,outline='black',width=3)
    d.text((cx,cy),str(n),fill=ink(c),font=fb,anchor='mm')
# buck pad tags, original-image coords: (pad x, pad y, net, tag centre y, colour)
ft=ImageFont.truetype(F,24)
A,B='#00B4FF','#7CFF00'
VTAGS=[(348,523,'GND',440,A),(712,523,'V_bat',440,A),(348,1336,'GND',1430,A),(712,1336,'12 V',1430,A),
       (148,235,'V_bat',150,'#FF2D2D'),(252,235,'GND',150,'#FF2D2D'),
       (600,385,'GND',250,'#111111'),
       (816,457,'GND',360,B),(1176,442,'V_bat',360,B),(816,1277,'GND',1385,B),(1176,1277,'3V3',1385,B)]
for px,py,l,ty,c in VTAGS:
    px+=OX
    d.line((px,py,px,ty),fill='black',width=5); d.line((px,py,px,ty),fill=c,width=2)
    d.ellipse((px-6,py-6,px+6,py+6),fill=c,outline='black',width=2)
    w=d.textlength(l,font=ft)
    d.rounded_rectangle((px-w/2-8,ty-17,px+w/2+8,ty+17),radius=6,fill=c,outline='black',width=2)
    d.text((px,ty),l,fill=ink(c),font=ft,anchor='mm')
# big dots on the eight buck pads, coloured by net
NET={'GND':'#111111','V_bat':'#FF2D2D','12 V':'#00B4FF','3V3':'#FF8C1A'}
for px,py,l,ty,c in VTAGS:
    if c in (A,B):
        x=px+OX; d.ellipse((x-17,py-17,x+17,py+17),fill=NET[l],outline='black' if l!='GND' else 'white',width=3)
# legend in the empty area below the board
ly0=2230
for i,(n,lab,b,c,_) in enumerate(E):
    x=60 if i<4 else im.width//2+40; y=ly0+(i%4)*56
    d.ellipse((x,y,x+44,y+44),fill=c,outline='black',width=2)
    d.text((x+22,y+22),str(n),fill=ink(c),font=fl,anchor='mm')
    d.text((x+58,y+22),lab,fill='#111111',font=fl,anchor='lm')
# key: every black wire / solder trace is GND
y=ly0+len(E)*56; x=60
d.rounded_rectangle((x,y+6,x+44,y+38),radius=6,fill='#111111',outline='black',width=2)
d.text((x+58,y+22),'Black wire / solder trace = GND (all of them)',fill='#111111',font=fl,anchor='lm')
im.save(DST,quality=92)
print(im.size)

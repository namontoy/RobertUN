# Annotated prototype photo: docs/Prototype_DriverWheels.jpg -> docs/Prototype_DriverWheels_boxes.jpg
# Run driverwheels_mask.py first if the mask is missing.
import os
from PIL import Image, ImageDraw, ImageFont
HERE=os.path.dirname(os.path.abspath(__file__))
SRC=HERE+'/../Prototype_DriverWheels.jpg'
DST=HERE+'/../Prototype_DriverWheels_boxes.jpg'
im=Image.open(SRC).convert('RGB')
# background -> neutral mid grey, using the GrabCut mask from mask.py
mask=Image.open(HERE+'/driverwheels_mask.png').convert('L')
im=Image.composite(im,Image.new('RGB',im.size,(128,128,128)),mask)
# widen the canvas: grey margins left/right; OX shifts every photo coordinate
OX,PADR=170,90
cv=Image.new('RGB',(im.width+OX+PADR,im.height),(128,128,128)); cv.paste(im,(OX,0)); im=cv
d=ImageDraw.Draw(im)
k=1512/1125
F='/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'
fb=ImageFont.truetype(F,34); fl=ImageFont.truetype(F,27)
# (n, label, display-coords box, colour, badge corner)
E=[
 (1,'DIP switch, module ID (MSB top)',          (80,340,234,520),  '#00B4FF','tl'),
 (2,'WeAct STM32F446RE core board (MCU)',     (290,296,780,914), '#FFFFFF','br'),
 (3,'Battery input, V_bat ~15 V',          (856,181,1034,520),'#FF2D2D','tl'),
 (4,'Rocker switch',                          (841,560,1004,785),'#FFA8A8','tl'),
 (5,'Steering servo (UART4, servo-side names)',                    (886,790,990,1050),'#00FFD0','tr'),
 (6,'CAN in, JST-XH 3-pin',               (36,596,137,780),  '#FF69B4','tl'),
 (7,'CAN out, JST-XH 3-pin',              (36,806,137,960),  '#B388FF','bl'),
 (8,'CAN transceiver',   (143,596,264,844), '#7CFF00','tr'),
 (9,'Encoder power (wheel motor)',            (116,1013,245,1114),'#4D7CFF','bl'),
 (10,'Encoder lines A/B (wheel motor)',          (266,1013,384,1120),'#FFF59D','bl'),
 (11,'DRV8874 breakout (PU = 10k pull-up)',        (396,943,664,1164),'#FF00FF','tr'),
 (12,'Motor output (DRV8874 OUT1/OUT2)',           (740,1040,904,1174),'#FFB300','tr'),
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
# pin tags, original-image coords: (pin x, pin y, label, tag x, colour)
ft=ImageFont.truetype(F,24)
TAGS=[(1252,y,l,1352,'#00FFD0') for y,l in
      [(1352,'V_bat'),(1302,'GND_1'),(1252,'TX→PA1'),(1209,'RX→PA0'),(1155,'GND_2'),(1110,'NC')]]
for px,py,l,tx,c in TAGS:
    px+=OX; tx+=OX
    d.line((px,py,tx,py),fill='black',width=5); d.line((px,py,tx,py),fill=c,width=2)
    d.ellipse((px-6,py-6,px+6,py+6),fill=c,outline='black',width=2)
    w=d.textlength(l,font=ft)
    d.rounded_rectangle((tx,py-17,tx+w+16,py+17),radius=6,fill=c,outline='black',width=2)
    d.text((tx+8,py),l,fill=ink(c),font=ft,anchor='lm')
# vertical pin tags: (pin x, pin y, label, tag centre y, colour)
VTAGS=[(197,1449,'GND',1565,'#4D7CFF'),(277,1449,'3V3',1565,'#FF8C1A'),
       (402,1440,'A→PA15',1565,'#FFF59D'),(470,1440,'B→PB3',1622,'#FFF59D'),
       (1059,1495,'OUT2',1625,'#FFB300'),(1159,1495,'OUT1',1625,'#FFB300'),
       (1234,360,'GND',190,'#FF2D2D'),(1331,360,'V_bat',190,'#FF2D2D')]
for px,py,l,ty,c in VTAGS:
    px+=OX
    d.line((px,py,px,ty),fill='black',width=5); d.line((px,py,px,ty),fill=c,width=2)
    d.ellipse((px-6,py-6,px+6,py+6),fill=c,outline='black',width=2)
    w=d.textlength(l,font=ft)
    d.rounded_rectangle((px-w/2-8,ty-17,px+w/2+8,ty+17),radius=6,fill=c,outline='black',width=2)
    d.text((px,ty),l,fill=ink(c),font=ft,anchor='mm')
# left tags, right edge at tx: (pin x, pin y, label, tag right x, colour)
LTAGS=[(192,y,l,25,'#00B4FF') for y,l in
       [(505,'PB15'),(554,'PB14'),(602,'PB13'),(650,'PB12')]]
for x,ys,c in [(122,(886,935,980),'#FF69B4'),(124,(1135,1187,1235),'#B388FF')]:
    LTAGS+=[(x,y,l,25,c) for y,l in zip(ys,('CANL','CANH','GND'))]
# transceiver: tags sit on the module body, smaller font
fs=ImageFont.truetype(F,21)
LTAGS=[t+(ft,) for t in LTAGS]+[(310,y,l,298,'#FF8C1A' if l=='3V3' else '#7CFF00',fs) for y,l in
       zip((855,904,952,1002,1050,1097),('CANL','CANH','CRX→PB8','CTX→PB9','GND','3V3'))]
for px,py,l,tx,c,fo in LTAGS:
    px+=OX; tx+=OX
    d.line((px,py,tx,py),fill='black',width=5); d.line((px,py,tx,py),fill=c,width=2)
    d.ellipse((px-6,py-6,px+6,py+6),fill=c,outline='black',width=2)
    w=d.textlength(l,font=fo); h=17 if fo is ft else 15
    d.rounded_rectangle((tx-w-14,py-h,tx,py+h),radius=6,fill=c,outline='black',width=2)
    d.text((tx-7,py),l,fill=ink(c),font=fo,anchor='rm')
# rotated tags hanging below a pin row: (pin x, pin y, label, tag top y, colour)
DTAGS=[(x,1537,l,1585,'#FF00FF') for x,l in zip((561,609,656,702,752,800,847),
       ('IPROPI→PA2','nFAULT→PB0','IOE (10k PU)','OUT1','OUT2','GND','VM'))]
# top row: no room above (core board), so these hang down over the breakout body
DTAGS+=[(x,1295,l,1345,'#FF00FF') for x,l in zip((567,615,664,711,760,806,857),
       ('VREF→PA4','SLP→PB5','PMODE (PU)','PH/IN2→PB7','EN/IN1→PB6','GND','VM'))]
for px,py,l,ty,c in DTAGS:
    px+=OX
    d.line((px,py,px,ty),fill='black',width=5); d.line((px,py,px,ty),fill=c,width=2)
    d.ellipse((px-6,py-6,px+6,py+6),fill=c,outline='black',width=2)
    fo=ft if py>1400 else fs; hh=34 if fo is ft else 30
    w=int(d.textlength(l,font=fo))+16
    t=Image.new('RGBA',(w,hh),(0,0,0,0)); td=ImageDraw.Draw(t)
    td.rounded_rectangle((0,0,w-1,hh-1),radius=6,fill=c,outline='black',width=2)
    td.text((w//2,hh//2),l,fill=ink(c),font=fo,anchor='mm')
    t=t.rotate(270,expand=True); im.paste(t,(px-hh//2,ty),t)
# 3V3 rail: orange wire left of the MCU, joined to the core board's 3V3 pin column (top right)
OR='#FF8C1A'
d.rectangle((995+OX,485,1038+OX,627),outline='black',width=6); d.rectangle((995+OX,485,1038+OX,627),outline=OR,width=3)
px,py,tx=1038+OX,556,1080+OX
d.line((px,py,tx,py),fill='black',width=5); d.line((px,py,tx,py),fill=OR,width=2)
w=d.textlength('3V3',font=ft)
d.rounded_rectangle((tx,py-17,tx+w+16,py+17),radius=6,fill=OR,outline='black',width=2)
d.text((tx+8,py),'3V3',fill=ink(OR),font=ft,anchor='lm')
t=Image.new('RGBA',(int(d.textlength('3V3 wire → 8, 9',font=fs))+16,30),(0,0,0,0)); td=ImageDraw.Draw(t)
td.rounded_rectangle((0,0,t.width-1,29),radius=6,fill=OR,outline='black',width=2)
td.text((t.width//2,15),'3V3 wire → 8, 9',fill=ink(OR),font=fs,anchor='mm')
t=t.rotate(270,expand=True); im.paste(t,(383+OX-15,900-t.height//2),t)
# MCU header dots, coloured like the connector each pin serves.
# Header columns (photo coords): left outer 420 / inner 470, right inner 962 / outer 1015.
# Rows every 50 px from y=500. Inner/outer per the user's check (B12 inner, B13 outer; C5 inner, B0 outer).
LO,LI,RI,RO=420,470,962,1015
def ry(i): return 500+50*i
DIP,SRV,DRV,ENC,CAN='#00B4FF','#00FFD0','#FF00FF','#FFF59D','#7CFF00'
DOTS=[(LI,ry(2),DIP),(LO,ry(2),DIP),(LI,ry(3),DIP),(LO,ry(3),DIP),      # PB12 PB13 PB14 PB15
      (LO,ry(8),ENC),(LI,ry(11),ENC),                                    # PA15 PB3
      (LI,ry(12),DRV),(LO,ry(12),DRV),(LI,ry(13),DRV),                   # PB5 PB6 PB7
      (LO,ry(13),CAN),(LI,ry(14),CAN),                                   # PB8 PB9
      (RO,ry(10),SRV),(RI,ry(9),SRV),                                    # PA0 PA1
      (RO,ry(9),DRV),(RO,ry(8),DRV),(RO,ry(5),DRV)]                      # PA2 PA4 PB0
for x,y,c in DOTS:
    x+=OX; d.ellipse((x-17,y-17,x+17,y+17),fill=c,outline="black",width=3)
# legend in the empty dark area below the board
ly0=2170; cols=[(60,0,6),(im.width//2+40,6,12)]
for x,a,z in cols:
    for i,(n,lab,b,c,_) in enumerate(E[a:z]):
        y=ly0+i*56
        d.ellipse((x,y,x+44,y+44),fill=c,outline='black',width=2)
        d.text((x+22,y+22),str(n),fill=ink(c),font=fl,anchor='mm')
        d.text((x+58,y+22),lab,fill='#111111',font=fl,anchor='lm')
# rail key under the first legend column
y=ly0+6*56; x=cols[0][0]
d.rounded_rectangle((x,y+6,x+44,y+38),radius=6,fill=OR,outline='black',width=2)
d.text((x+58,y+22),'3V3 rail (orange): MCU 3V3 → wire → 8, 9.  Dots on 2 = pin used, connector colour',fill='#111111',font=fl,anchor='lm')
im.save(DST,quality=92)
print(im.size)

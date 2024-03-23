import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets
import pyqtgraph.opengl as gl
from stl import mesh
import serial
import serial.tools.list_ports
import struct
from math import isclose

ports = serial.tools.list_ports.comports()
ser = serial.Serial(        #define COM port parameters except port
     baudrate=2000000,
     parity=serial.PARITY_NONE,
     stopbits=serial.STOPBITS_ONE,
     bytesize=serial.EIGHTBITS,
     timeout=None)

data_out = []       #serial data package
comm_flag = False
send_flag = False
mode = 1            #initial mode 1 - control, 2 - dynamics, 3 - haptics, 4 - model w/ impulse

k = 0   #stiffness
c = 0   #damping coeff.
d = 10  #virtual wall distance

kmax = 200  #N/m (nominal 80 N/m)
cmax = 5    #N.s/m (nominal 1.5 N.s/m)

kp = 0 #proportional gain
ki = 0 #integral gain
kd = 0 #derivative gain

kpmax = 15; kimax = 10; kdmax = 3

t = np.linspace(0, 5, num=250, endpoint=False)           #time vector
th_r = np.zeros(len(t)).astype(int)        #target angular position
th = np.zeros(len(t))           #angular position
om = np.zeros(len(t))           #angular velocity

last_th = 0                     #a variable to store the last angular position of the handle
last_x = 0
last_sp = 60
last_d = d
t_old = 0
x = np.zeros(len(t))            #horizontal position
v = np.zeros(len(t))            #horizontal velocity
F = np.zeros(len(t))            #horizontal force

rh = 50         #handle radius in mm
ra = 75 + 2     #arc radius in mm
rp = 6          #pulley radius in mm

comm_interval = 10

def comm():
    global ser, data_out, t, x ,v, th, om, F, last_th, last_x, t_old, send_flag
    if comm_flag:
        if send_flag == True:
            data_out=struct.pack("bbhhhhhb", mode, th_r[0], kp, ki, kd, k, c, d)
            ser.write(data_out)
            send_flag = False
        if ser.in_waiting > 0:
            data_in=ser.read(8)
        else:
            data_in = ""
        
        if data_in:
            dummy = struct.unpack("ff", data_in)
            th[0] = dummy[0]#*180/3.14   #angular position in deg
            om[0] = dummy[1]            #angular velocity in rad/s
            x[0]=dummy[0]*rh            #horizontal position in mm
            v[0]=om[0]*rh/1000          #horizontal velocity in m/s
            if abs(x[0]) > (d/2):
                F[0]=k/1000*(np.copysign(d, x[0])/2-x[0])-c/100*v[0]     #estimated force in N
            else:
                F[0]=0

            th = np.roll(th, -1)
            om = np.roll(om, -1)
            x = np.roll(x, -1)
            v = np.roll(v, -1)
            F = np.roll(F, -1)
        render()

def render():
    global last_th, last_x, last_sp, render_count
    if (tabs.currentIndex() == 0) or (tabs.currentIndex() == 3):
        pos_label.setText("  "+u"\u03b8 = "+ str(round(th[249],3)) + " rad = " + str(round(th[249]*180/3.14,2)) + " \u00b0")
        data_pos.setData(t, th)
        data_ref.setData(t, th_r/100)
        data_vel.setData(t, om)
        p_top.setYRange(-1, 1)
        p_top.setLabel('left', "angular position", units="rad")
        p_mid.setYRange(-20, 20)
        p_mid.setLabel('left', "angular velocity", units="rad/s")
    else:
        pos_label.setText("x = "+ str(round(x[249],2)) + " mm")
        data_pos.setData(t, x)
        data_vel.setData(t, v)
        data_for.setData(t, F)
        p_top.setYRange(-55, 55)
        p_top.setLabel('left', "position", units="mm")
        p_mid.setYRange(-1, 1)
        p_mid.setLabel('left', "velocity", units="m/s")
        p_bot.setLabel('left', "force", units="N")

    if not (th[249] == last_th):
        sector.rotate((-th[249]+last_th)*180/3.14,1,0,0,False)
        ball.translate(0,x[249]-last_x,0)
        block.translate(0,(x[249]-last_x)*0.7,0)
        wheels.translate(0,(x[249]-last_x)*0.7,0)
        spring.scale((last_sp+(x[249]-last_x)*0.7)/last_sp,1,1)
        damper_r.scale(1,1,(last_sp+(x[249]-last_x)*0.7)/last_sp)

    last_sp = last_sp + (x[249] - last_x)*0.7
    last_th = th[249]
    last_x = x[249]

def con_sliders():
    global kp, ki, kd, th_r, send_flag
    th_r[:] = sl_tar.value()
    kp = sl_kp.value()
    ki = sl_ki.value()
    kd = sl_kd.value()

    t_tar.setText(str(th_r[0]/100))
    t_kp.setText(str(kp/10))
    t_ki.setText(str(ki/10))
    t_kd.setText(str(kd/10))

def con_start():
    global mode, send_flag
    mode = 1
    send_flag = True

def con_rstgains():
    global kp, ki, kd, th_r, mode, send_flag
    sl_tar.setValue(0)
    sl_kp.setValue(0)
    sl_ki.setValue(0)
    sl_kd.setValue(0)
    mode = 1
    send_flag = True

def dynhap_sliders():
    global d, k, c, last_d, send_flag
    if mode == 3:
        d = sl_wd.value()
        t_wd.setText(str(d)+' mm')
        wall_l.translate(0,(last_d-d)/2,0)
        wall_r.translate(0,-(last_d-d)/2,0)
        last_d = d
    
    k = sl_k.value()
    c = sl_c.value()

    t_k.setText(str(k)+' N/m')
    t_c.setText(str(c/100)+' N.s/m')    # Slider value is c*100 (float to int conversion)
   
    send_flag = True

def mod_start():
    global mode, send_flag
    mode = 4
    send_flag = True
    timer_mod.start(3100)

def model():
    timer_mod.stop()
    opencom()
    flats = []
    jumps = np.zeros(6)
    gains =[]
    taus = np.zeros(6)
    gain = 0
    tau_ =  0
    tau = 0
    
    for i in range(90,250,25):
        flats.append(th[i])

    s = 90
    for i in range(6):
        gains.append(flats[i+1]-flats[i])
        tau_=flats[i] + 0.632 * gains[i]
        for d in range(s, s+25):
            if (isclose(th[d], flats[i], abs_tol = 0.003)) and (abs(th[d+1]-th[d]) > 0.008):
                jumps[i]=t[d]
            if ((th[d] < tau_) and (th[d+1] > tau_)) or ((th[d] > tau_) and (th[d+1] < tau_)):
                taus[i] = (tau_ - th[d])/(th[d+1]-th[d])*(t[d+1]-t[d])+t[d]
        s = s + 25

    j = 0
    for i in range(6):
        if (taus[i] != 0) and (jumps[i] != 0):
            tau = tau + (taus[i]-jumps[i])
            j = j + 1
    
    gain = round(sum(np.absolute(gains))/len(gains)/9/0.01, 3) #divide the gain to (9V*0.01s)
    tau = round(tau/j, 3)
    l_tf.setText("\u03f4(s)           "+str(gain)+" \u2800 \n ------- = -------------- \n  V(s)       "+str(tau)+"s + 1" )
 
def opencom():
    global ser, comm_flag
    if not combo_com.currentText() == "":
        if b_com.text() == "Connect":
            ser.port = combo_com.currentText()
            ser.open()
            comm_flag = True
            b_com.setText("Disconnect")
            b_com.setStyleSheet('background-color: red;')
            b_imp.setEnabled(True)
        else:
            ser.reset_input_buffer()
            ser.close()
            comm_flag = False
            b_com.setText("Connect")
            b_com.setStyleSheet('background-color: lightgreen;')
            b_imp.setEnabled(False)

def resetview():
    aw.setCameraPosition(pos=pg.QtGui.QVector3D(0, 0, -20), distance=200, azimuth=0, elevation=0)

def resetpaddle():
    global mode, send_flag
    if comm_flag == True:
        mode = 0
        send_flag = True

def tabSelected(arg=None):
    global d, mode, send_flag
    aw.clear()
    if arg==0:
        mode = 1
        aw.addItem(sector); aw.addItem(pole); aw.addItem(base); aw.addItem(motor); aw.addItem(roller)
        if pw.getItem(3,0):
            pw.removeItem(p_bot)
        send_flag = False
    if arg==1:
        mode = 2
        aw.addItem(ground); aw.addItem(block); aw.addItem(spring); aw.addItem(damper_l); aw.addItem(damper_r); aw.addItem(wheels)
        pw.addItem(p_bot, 3, 0)
        dynlayout.addWidget(sl_k, 0, 1)
        dynlayout.addWidget(t_k, 0, 2)
        dynlayout.addWidget(sl_c, 1, 1)
        dynlayout.addWidget(t_c, 1, 2)
        d = 0
        send_flag = True
    if arg==2:
        mode = 3
        aw.addItem(ball); aw.addItem(wall_l); aw.addItem(wall_r)
        pw.addItem(p_bot, 3, 0)
        haplayout.addWidget(sl_k, 1, 1)
        haplayout.addWidget(t_k, 1, 2)
        haplayout.addWidget(sl_c, 2, 1)
        haplayout.addWidget(t_c, 2, 2)
        d = last_d
        send_flag = True
    if arg==3:
        aw.addItem(sector); aw.addItem(pole); aw.addItem(base); aw.addItem(motor); aw.addItem(roller)
        if pw.getItem(3,0):
            pw.removeItem(p_bot)
        send_flag = True

################################################################################
##### GENERAL LAYOUT ###########################################################
################################################################################

app = pg.mkQApp()
mw = QtWidgets.QMainWindow()
mw.setWindowTitle('TEDUPaddle')
mw.setFixedSize(1000,600)
aw = gl.GLViewWidget()
aw.setFixedSize(360,360)
pw = pg.GraphicsLayoutWidget()

pagelayout = QtWidgets.QHBoxLayout()
leftlayout = QtWidgets.QVBoxLayout()
setuplayout = QtWidgets.QHBoxLayout()

conlayout = QtWidgets.QGridLayout()
dynlayout = QtWidgets.QGridLayout()
haplayout = QtWidgets.QGridLayout()
modlayout = QtWidgets.QGridLayout()

tabs = QtWidgets.QTabWidget()
tabs.setTabPosition(QtWidgets.QTabWidget.North)

tab_con = QtWidgets.QWidget()
tab_con.setLayout(conlayout)
tab_dyn = QtWidgets.QWidget()
tab_dyn.setLayout(dynlayout)
tab_hap = QtWidgets.QWidget()
tab_hap.setLayout(haplayout)
tab_mod = QtWidgets.QWidget()
tab_mod.setLayout(modlayout)

tabs.addTab(tab_con, "Control")
tabs.addTab(tab_dyn, "Dynamics")
tabs.addTab(tab_hap, "Haptics")
tabs.addTab(tab_mod, "Modeling")

combo_com = QtWidgets.QComboBox()

for p in ports:
    combo_com.addItem(p.device)
b_com = QtWidgets.QPushButton("Connect")
b_com.setStyleSheet('background-color: lightgreen;')
b_com.clicked.connect(opencom)
b_view = QtWidgets.QPushButton("Reset View")
b_view.setStyleSheet('background-color: lightblue;')
b_view.clicked.connect(resetview)
b_reset = QtWidgets.QPushButton("Reset Paddle")
b_reset.setStyleSheet('background-color: lightcoral;')

b_reset.clicked.connect(resetpaddle)

setuplayout.addWidget(combo_com)
setuplayout.addWidget(b_com)
setuplayout.addWidget(b_view)
setuplayout.addWidget(b_reset)

leftlayout.addWidget(aw)
leftlayout.addLayout(setuplayout)
leftlayout.addWidget(tabs)

pagelayout.addLayout(leftlayout)
pagelayout.addWidget(pw)

cw = QtWidgets.QWidget()
cw.setLayout(pagelayout)
mw.setCentralWidget(cw)

################################################################################
##### CONTROL TAB ##############################################################
################################################################################

sl_tar = QtWidgets.QSlider(QtCore.Qt.Horizontal)
sl_kp = QtWidgets.QSlider(QtCore.Qt.Horizontal)
sl_ki = QtWidgets.QSlider(QtCore.Qt.Horizontal)
sl_kd = QtWidgets.QSlider(QtCore.Qt.Horizontal)

sl_k = QtWidgets.QSlider(QtCore.Qt.Horizontal)
sl_c = QtWidgets.QSlider(QtCore.Qt.Horizontal)
sl_wd = QtWidgets.QSlider(QtCore.Qt.Horizontal)

l_tar = QtWidgets.QLabel("Target position (" + u"\u03b8r)")
l_kp = QtWidgets.QLabel("Porportional gain (Kp)")
l_ki = QtWidgets.QLabel("Integral gain (Ki)")
l_kd = QtWidgets.QLabel("Derivative gain (Kd)")

l_k = QtWidgets.QLabel("Stiffness (k)")
l_c = QtWidgets.QLabel("Damping (c)")

l_wk = QtWidgets.QLabel("Wall stiffness (k)")
l_wc = QtWidgets.QLabel("Wall damping (c)")
l_wd = QtWidgets.QLabel("Wall distance (d)")
l_dummyd = QtWidgets.QLabel("")
l_dummyh = QtWidgets.QLabel("")

ss1 = 'color: black; background-color: white; font: bold;'
t_tar = QtWidgets.QLineEdit(); t_tar.setAlignment(QtCore.Qt.AlignHCenter); t_tar.setEnabled(False); t_tar.setStyleSheet(ss1)
t_kp = QtWidgets.QLineEdit(); t_kp.setAlignment(QtCore.Qt.AlignHCenter); t_kp.setEnabled(False); t_kp.setStyleSheet(ss1)
t_ki = QtWidgets.QLineEdit(); t_ki.setAlignment(QtCore.Qt.AlignHCenter); t_ki.setEnabled(False); t_ki.setStyleSheet(ss1)
t_kd = QtWidgets.QLineEdit(); t_kd.setAlignment(QtCore.Qt.AlignHCenter); t_kd.setEnabled(False); t_kd.setStyleSheet(ss1)
t_k = QtWidgets.QLineEdit(); t_k.setAlignment(QtCore.Qt.AlignRight); t_k.setEnabled(False); t_k.setStyleSheet(ss1)
t_c = QtWidgets.QLineEdit(); t_c.setAlignment(QtCore.Qt.AlignRight); t_c.setEnabled(False); t_c.setStyleSheet(ss1)
t_wd = QtWidgets.QLineEdit(); t_wd.setAlignment(QtCore.Qt.AlignRight); t_wd.setEnabled(False); t_wd.setStyleSheet(ss1)

t_tar.setFixedSize(50,20);t_kp.setFixedSize(50,20);t_ki.setFixedSize(50,20);t_kd.setFixedSize(50,20)
t_k.setFixedSize(70,20);t_c.setFixedSize(70,20);t_wd.setFixedSize(70,20)
t_tar.setText(str(th_r[0]));t_kp.setText(str(kp));t_ki.setText(str(ki));t_kd.setText(str(kd))
t_k.setText(str(k)+'N/m');t_c.setText(str(c));t_wd.setText(str(d))

b_start = QtWidgets.QPushButton("Start")
b_start.clicked.connect(con_start)

b_rstgains = QtWidgets.QPushButton("Reset Gains")
b_rstgains.clicked.connect(con_rstgains)

b_imp = QtWidgets.QPushButton("Start")
b_imp.clicked.connect(mod_start)
b_imp.setEnabled(False)

l_tf = QtWidgets.QLabel("\u03f4(s)               K   \u2800 \n ------- = -------------- \n V(s)           \u03c4s + 1  " )
l_tf.setWordWrap=True
l_tf.setAlignment(QtCore.Qt.AlignHCenter | QtCore.Qt.AlignVCenter)
l_tf.setStyleSheet('color: black; background-color: lightblue; font: bold 16px;')

sl_tar.setRange(-75,75); sl_tar.setValue(th_r[0]); sl_tar.setSingleStep(1); sl_tar.setPageStep(5)
sl_kp.setRange(0,kpmax*10); sl_kp.setValue(kp); sl_kp.setSingleStep(1); sl_kp.setPageStep(5)
sl_ki.setRange(0,kimax*10); sl_ki.setValue(ki); sl_ki.setSingleStep(1); sl_ki.setPageStep(5)
sl_kd.setRange(0,kdmax*10); sl_kd.setValue(kd); sl_kd.setSingleStep(1); sl_kd.setPageStep(5)
sl_tar.valueChanged.connect(con_sliders); sl_kp.valueChanged.connect(con_sliders)
sl_ki.valueChanged.connect(con_sliders); sl_kd.valueChanged.connect(con_sliders)

sl_wd.setRange(10,80); sl_wd.setValue(d); sl_wd.setSingleStep(1); sl_wd.setPageStep(5)
sl_k.setRange(0,kmax); sl_k.setValue(0); sl_k.setSingleStep(1); sl_k.setPageStep(5)
sl_c.setRange(0,cmax*100); sl_c.setValue(0); sl_c.setSingleStep(1); sl_c.setPageStep(10)
sl_wd.valueChanged.connect(dynhap_sliders); sl_k.valueChanged.connect(dynhap_sliders); sl_c.valueChanged.connect(dynhap_sliders)

conlayout.addWidget(l_tar, 0, 0)
conlayout.addWidget(sl_tar, 0, 1)
conlayout.addWidget(t_tar, 0, 2)
conlayout.addWidget(l_kp, 1, 0)
conlayout.addWidget(sl_kp, 1, 1)
conlayout.addWidget(t_kp, 1, 2)
conlayout.addWidget(l_ki, 2, 0)
conlayout.addWidget(sl_ki, 2, 1)
conlayout.addWidget(t_ki, 2, 2)
conlayout.addWidget(l_kd, 3, 0)
conlayout.addWidget(sl_kd, 3, 1)
conlayout.addWidget(t_kd, 3, 2)
conlayout.addWidget(b_rstgains, 4, 0, 4, 1)
conlayout.addWidget(b_start, 4, 1, 4, 3)

dynlayout.addWidget(l_k, 0, 0)
dynlayout.addWidget(l_c, 1, 0)
dynlayout.addWidget(l_dummyd, 2, 0, 4, 3)

haplayout.addWidget(l_wd, 0, 0)
haplayout.addWidget(sl_wd, 0, 1)
haplayout.addWidget(t_wd, 0, 2)
haplayout.addWidget(l_wk, 1, 0)
haplayout.addWidget(sl_k, 1, 1)
haplayout.addWidget(t_k, 1, 2)
haplayout.addWidget(l_wc, 2, 0)
haplayout.addWidget(sl_c, 2, 1)
haplayout.addWidget(t_c, 2, 2)
haplayout.addWidget(l_dummyh, 3, 0, 4, 3)

modlayout.addWidget(b_imp, 1, 0)
modlayout.addWidget(l_tf, 2, 0)

################################################################################
##### PLOT WINDOW ##############################################################
################################################################################

#pg.setConfigOptions(antialias=True)
pos_label = pg.LabelItem(justify='right')
pw.addItem(pos_label)
pw.nextRow()
p_top = pw.addPlot()
p_top.showGrid(x=True, y=True)
p_top.setXRange(0,5)
p_top.setYRange(-1,1)

p_top.disableAutoRange()
pw.nextRow()
p_mid = pw.addPlot()
p_mid.showGrid(x=True, y=True)
p_mid.setXRange(0,5)
p_mid.disableAutoRange()
pw.nextRow()
p_bot = pw.addPlot()
p_bot.showGrid(x=True, y=True)
p_bot.setXRange(0,5)
p_bot.setYRange(-5.1,5.1)
p_bot.disableAutoRange()
pw.nextRow()
pw.addItem(pg.LabelItem(text="time (s)", justify="center"))

data_pos = p_top.plot(pen='y')
data_ref = p_top.plot(pen='r')
data_vel = p_mid.plot(pen='y')
data_for = p_bot.plot(pen='y')

################################################################################
##### ANIMATION WINDOW #########################################################
################################################################################

stls = ['stl/rotor_gui.stl', 'stl/pole.stl', 'stl/base.stl', 'stl/motor.stl', 'stl/roller.stl', 'stl/wall.stl', 'stl/ground.stl', 'stl/block.stl', 'stl/spring.stl', 'stl/wheels.stl']
mds = []
for stl in stls:
    meshdata = mesh.Mesh.from_file(stl)
    pts = meshdata.points.reshape(-1,3)
    pts[:,[2,0]]=pts[:,[0,2]]
    fcs = np.arange(pts.shape[0]).reshape(-1,3)
    mds.append(gl.MeshData(vertexes=pts, faces=fcs))

mds.append(gl.MeshData.sphere(rows=10, cols=10, radius=2))
mds.append(gl.MeshData.cylinder(rows=10, cols=10, radius=4, length=35))
mds.append(gl.MeshData.cylinder(rows=10, cols=10, radius=2, length=70))

sector = gl.GLMeshItem(meshdata=mds[0], smooth=False, drawFaces=True, shader="shaded", drawEdges=False, color=(0.9,0,0,0))
sector.rotate(90,1,0,0,False)
sector.rotate(180,0,0,1,False)
sector.translate(0,0,0)

pole = gl.GLMeshItem(meshdata=mds[1], smooth=False, drawFaces=True, shader="shaded", drawEdges=False, color=(0,0,0.9,0))
pole.rotate(90,1,0,0,False)
pole.translate(-35,0,-94)

base = gl.GLMeshItem(meshdata=mds[2], smooth=False, drawFaces=True, shader="shaded", drawEdges=False, color=(0.9,0.9,0,0))
base.rotate(90,1,0,0,False)
base.rotate(-90,0,1,0,False)
base.translate(15,50,-99)
base.scale(x=1, y=1, z=1)

motor = gl.GLMeshItem(meshdata=mds[3], smooth=False, drawFaces=True, shader="shaded", drawEdges=False, color=(0.7,0.7,0.7,0))
motor.rotate(90,1,0,0,False)
motor.translate(-45,0,-81.5)

roller = gl.GLMeshItem(meshdata=mds[4], smooth=False, drawFaces=True, shader="shaded", drawEdges=False, color=(0.9,0,0,0))
roller.rotate(90,1,0,0,False)
roller.translate(-11,0,-81.5)

wall_l = gl.GLMeshItem(meshdata=mds[5], smooth=False, drawFaces=True, shader="balloon", drawEdges=False, color=(0.8,0.8,0,0.7), glOptions='translucent')
wall_r = gl.GLMeshItem(meshdata=mds[5], smooth=False, drawFaces=True, shader="balloon", drawEdges=False, color=(0.8,0.8,0,0.7), glOptions='translucent')
wall_l.scale(20,0.2,1)
wall_r.scale(20,0.2,1)

wall_l.rotate(90,0,0,1,False)
wall_l.translate(120,-205,-138)
wall_r.rotate(90,0,0,1,False)
wall_r.translate(120.5,5,-138)

ground = gl.GLMeshItem(meshdata=mds[6], smooth=False, drawFaces=True, shader="shaded", drawEdges=False, color=(1,1,1,0))
ground.rotate(180,0,0,1,False)
ground.rotate(-90,1,0,0,False)
ground.translate(70,-70, -40)
ground.scale(1,1,1.2)

block = gl.GLMeshItem(meshdata=mds[7], smooth=False, drawFaces=True, shader="shaded", drawEdges=False, color=(0.9,0,0,0))
block.rotate(-90,1,0,0,local=False)
block.translate(45,0,-1,False)

spring = gl.GLMeshItem(meshdata=mds[8], smooth=False, drawFaces=True, shader="shaded", drawEdges=False, color=(0,0,0.9,0))
spring.rotate(90,0,0,1,False)
spring.translate(55, -68, -17.5, False)

wheels = gl.GLMeshItem(meshdata=mds[9], smooth=True, drawFaces=True, shader="shaded", drawEdges=False, color=(0,0,0.9,0))
wheels.rotate(90,1,0,0,False)
wheels.translate(47, 18, -33, False)

ball = gl.GLMeshItem(meshdata=mds[10], smooth=True, drawFaces=True, shader=None, drawEdges=False, color=(0.8,0,0,0))
ball.translate(98, 0,-25)

damper_l = gl.GLMeshItem(meshdata=mds[11], smooth=True, drawFaces=True, shader="shaded", drawEdges=False, color=(0.6,0.6,0,0))
damper_l.rotate(-90,1,0,0)
damper_l.translate(55, -68, -17.5, False)
damper_r = gl.GLMeshItem(meshdata=mds[12], smooth=True, drawFaces=True, shader="shaded", drawEdges=False, color=(0.9,0.9,0,0))
damper_r.rotate(-90,1,0,0)
damper_r.translate(55, -68, -17.5, False)

aw.addItem(sector); aw.addItem(pole); aw.addItem(base); aw.addItem(motor); aw.addItem(roller); pw.removeItem(p_bot)

tabs.currentChanged.connect(tabSelected) 
aw.setCameraPosition(pos=pg.QtGui.QVector3D(0, 0, -20), distance=200, azimuth=0, elevation=0)
aw.setBackgroundColor(112,128,144)#(255,255,255)

timer_comm = QtCore.QTimer()
timer_comm.timeout.connect(comm)
timer_comm.start(comm_interval)

timer_mod = QtCore.QTimer()
timer_mod.timeout.connect(model)

mw.show()
if __name__ == '__main__':
    pg.exec()

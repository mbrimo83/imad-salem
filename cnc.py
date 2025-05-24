# -*- coding: utf-8 -*-
"""
Created on Thu Jun 13 20:42:34 2024

@author: mohamed
"""
import serial
from serial.tools import list_ports
import sys
from PyQt5.QtWidgets import*
from PyQt5.QtGui import QColor
from PyQt5 import uic
from PyQt5.QtCore import QTimer,QThread, pyqtSignal
import re
import math
from PyQt5.QtOpenGL import QGLWidget
from OpenGL.GL import *
from OpenGL.GLU import *
from PyQt5.QtOpenGL import QGLWidget
from PyQt5.QtCore import Qt
from OpenGL.GL import *
from OpenGL.GLU import *
import time




class GCodeSender(QThread):
    progress = pyqtSignal(int)
    finished = pyqtSignal()
    error = pyqtSignal(str)
    lineProcessed = pyqtSignal(int)

    def __init__(self, ser, glines):
        super().__init__()
        self.ser = ser
        self.glines = glines
        self.current_line_index = 0
        self._continue = True
        self.paused = False
        self.waiting_for_response = False
        self.timeout_occurred = False

        self.timeout_timer = QTimer()
        self.timeout_timer.setSingleShot(True)
        self.timeout_timer.timeout.connect(self.handle_timeout)

        self.check_timer = QTimer()
        self.check_timer.setInterval(10)
        self.check_timer.timeout.connect(self.check_serial)

    def start(self):
        if not self.ser or not self.ser.is_open:
            self.error.emit("Serial port is not open.")
            return
        self._continue = True
        self.send_next_line()

    def stop(self):
        self._continue = False
        self.timeout_timer.stop()
        self.check_timer.stop()

    def pause(self):
        self.paused = True
        self.timeout_timer.stop()
        self.check_timer.stop()

    def resume(self):
        self.paused = False
        # resend the same line
        self.send_next_line()

    def send_next_line(self):
        if not self._continue or self.current_line_index >= len(self.glines):
            self.finished.emit()
            return

        if self.paused:
            return  # Wait for resume

        try:
            line = self.glines[self.current_line_index]
            self.ser.write((line.strip() + '\n').encode())
        except Exception as e:
            self.error.emit(f"Failed to write to serial port: {str(e)}")
            self.stop()
            return

        self.waiting_for_response = True
        self.timeout_occurred = False
        self.timeout_timer.start(5000)
        self.check_timer.start()

    def check_serial(self):
        if self.paused or not self.waiting_for_response or not self._continue:
            return

        try:
            if self.ser.in_waiting:
                response = self.ser.readline().decode().strip()
                if response == "$":
                    self.timeout_timer.stop()
                    self.check_timer.stop()
                    self.waiting_for_response = False

                    self.progress.emit(int((self.current_line_index + 1) * 100 / len(self.glines)))
                    self.lineProcessed.emit(self.current_line_index + 1)
                    self.current_line_index += 1
                    self.send_next_line()
        except Exception as e:
            self.error.emit(f"Serial error while waiting for response: {str(e)}")
            self.stop()

    def handle_timeout(self):
        if not self.timeout_occurred:
            self.timeout_occurred = True
            self.error.emit(f"Timeout waiting for response at line {self.current_line_index + 1}")



class OpenGLWidget(QGLWidget):
    def __init__(self, parent=None):
        super(OpenGLWidget, self).__init__(parent)
        self.shapes = []
        self.arc_params_list = []
        self.line_params_list = []
        self.center = (30, 30)  # Set the new origin

        # Zoom and Pan state
        self.zoom = 1.0
        self.pan_x = 0.0
        self.pan_y = 0.0
        self.last_mouse_pos = None
        self.xc=0
        self.yc=0

    def initializeGL(self):
        glClearColor(0.0, 0.0, 0.0, 1.0)  # Black background
        glClear(GL_COLOR_BUFFER_BIT)
        glLoadIdentity()

    def resizeGL(self, w, h):
        glViewport(0, 0, w, h)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        aspect_ratio = w / h if h != 0 else 1
        if aspect_ratio > 1:
            gluOrtho2D(0, 300 * aspect_ratio, 0, 300)
        else:
            gluOrtho2D(0, 300, 0, 300 / aspect_ratio)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

    def paintGL(self):
        glClear(GL_COLOR_BUFFER_BIT)
        glLoadIdentity()

        # Apply zoom and pan
        glScalef(self.zoom, self.zoom, 1.0)
        glTranslatef(self.pan_x, self.pan_y, 0.0)

        self.draw_axes()
        self.draw_cross_pointer((20, 20), 20)

        for shape in self.shapes:
            shape()
        glFlush()

    

    def wheelEvent(self, event):
        delta = event.angleDelta().y()
        zoom_factor = 1.1
        if delta > 0:
            self.zoom *= zoom_factor
        else:
            self.zoom /= zoom_factor
        self.update()

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.last_mouse_pos = event.pos()

    def mouseMoveEvent(self, event):
        if event.buttons() & Qt.LeftButton and self.last_mouse_pos is not None:
            dx = event.x() - self.last_mouse_pos.x()
            dy = event.y() - self.last_mouse_pos.y()
            self.pan_x += dx / self.zoom
            self.pan_y -= dy / self.zoom
            self.last_mouse_pos = event.pos()
            self.update()

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.last_mouse_pos = None


    def draw_cross_pointer(self, pcenter=(0, 0), size=20, color=(0.0, 1.0, 0.0)):
        half = size / 2
        glColor3f(*color)
        glBegin(GL_LINES)
        # Horizontal line
        glVertex2f(pcenter[0]+self.center[0]- half, pcenter[1]+self.center[1])
        glVertex2f(pcenter[0]+self.center[0] + half, pcenter[1]+self.center[1])
        # Vertical line
        glVertex2f(pcenter[0]+self.center[0], self.center[1]+pcenter[1] - half)
        glVertex2f(pcenter[0]+self.center[0], self.center[1]+pcenter[1] + half)
        glEnd()

    def draw_axes(self):
        glColor3f(1.0, 0.0, 0.0)  # Red color
        glBegin(GL_LINES)
        # X axis
        glVertex2f(0, self.center[1])
        glVertex2f(self.center[0] + 1000.0, self.center[1])
        # Y axis
        glVertex2f(self.center[0], 0)
        glVertex2f(self.center[0], self.center[1] + 1000.0)
        glEnd()

    def draw_arc_method(self, arc_params):
        direction, x1, y1, xcenter, ycenter, radius, start_angle, end_angle, color = arc_params
        glColor3f(*color)  # Set color
        
        self.xc = self.center[0] +xcenter
        self.yc = self.center[1] +ycenter
        theta1 = start_angle
        theta2 = end_angle
        #Recap of angle difference formulas:
        #G3 (CCW):

        
        if direction==3 :
            theta = (theta2 - theta1 + 2 * math.pi) % (2 * math.pi)

        #G2 (CW):
        if direction==2:
            theta = (theta1 - theta2 + math.pi*2) % (2 * math.pi)
        

        

        
        segments = 100
        diff = theta / segments
        
        glBegin(GL_LINE_STRIP) 
        if direction == 2:  # G2 - Clockwise
            for ir in range(segments + 1):
                current_angle = theta1 - ir * diff
                x = self.xc + radius * math.cos(current_angle)
                y = self.yc + radius * math.sin(current_angle)
                glVertex2f(x, y)
                # Optional debug:
                

        if direction == 3:  # G3 - Counterclockwise
            for ir in range(0,segments+1):
                current_angle = theta1 + ir * diff
                x = self.xc + radius * math.cos(current_angle)
                y = self.yc + radius* math.sin(current_angle)
                
                glVertex2f( x, y)
                

                # Optional debug:
                
        glEnd()

        
    def draw_line_method(self, line_params):
        start, end, color = line_params
        start = (start[0] + self.center[0], start[1] + self.center[1])
        end = (end[0] + self.center[0], end[1] + self.center[1])
        glColor3f(*color)  # Unpack RGB color
        glBegin(GL_LINES)
        glVertex2f(start[0], start[1])
        glVertex2f(end[0], end[1])
        glEnd()


    def set_arc_parameters(self, direction, px, py, xcenter, ycenter, radius, start_angle, end_angle, color=(1.0, 1.0, 1.0)):
        self.arc_params_list.append((direction, px, py, xcenter, ycenter, radius, start_angle, end_angle, color))
        self.shapes.append(lambda: self.draw_arc_method((direction, px, py, xcenter, ycenter, radius, start_angle, end_angle, color)))

    

    def set_line_parameters(self, start, end, color=(1.0, 1.0, 1.0)):  # Default white
        self.line_params_list.append((start, end, color))
        self.shapes.append(lambda: self.draw_line_method((start, end, color)))


    def toggle_arc(self):
        self.update()

    def toggle_line(self):
        self.update()

    def clear_drawing(self):
        self.shapes.clear()
        self.arc_params_list.clear()
        self.line_params_list.clear()
        self.update()
    def clear_shapes(self):
        self.shapes.clear()
        self.line_params_list.clear()
        self.arc_params_list.clear()
        self.update()  # Redraw the cleared OpenGL widget



            
class MyMainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        uic.loadUi('cnc.ui',self)
        self.settingsWidget.hide() 
        QMainWindow.setWindowTitle(self, "CNC Control v1.0")
        #QMainWindow.setFixedSize(self, 1350, 600)
        self.opengl_widget = OpenGLWidget(self)
        #self.opengl_widget.setMinimumSize(700, 500)

        layout = QVBoxLayout()
        self.openGLFrame.setLayout(layout)
        layout.setContentsMargins(0, 0, 0, 0)  # Optional: no spacing around widget
        layout.addWidget(self.opengl_widget)
        self.listWidget.itemClicked.connect(self.on_item_clicked)

        
        self.init()

    def init(self):
        self.jog_axis = None
        self.jog_sent = False


        self.ready_received= False
        self.px = 0
        self.py = 0
        self.pz = 0
        self.newx = 0
        self.newy = 0
        self.newz = 0
        self.dx = 0
        self.dy = 0
        self.dz = 0
        self.xsteps = 0
        self.ysteps = 0
        self.xstepfactor = 1  # Initialize with default value
        self.ystepfactor = 1  # Initialize with default value
        self.pre_frate = 100 # Default feed rate
        self.cur_frate=0
        self.counter=0
        self.CoList=[]
        self.garc=0
        self.r=0
        self.theta1=0
        self.theta2=0
        self.i=0
        self.j=0
        self.cx=0
        self.cy=0
        self.g=0
        self.send=0
        self.ir=0
        self.q=0
        self.glines=[]
        self.selected_line=' '
        self.move_time =0
        self.xy_time=0 
        self.z_time=0
        self.total_work_time=0
        self.hour=0
        self.minutes=0
        self.seconds=0
        self.total_work_time = 0
        self.com='COM1'
        self.baude='9600'
        #self.listWidget = QListWidget(self)       
        self.openact.setShortcut('Ctrl+O')
        self.progressBar.setValue(0)
        self.openact.triggered.connect(self.openFileDialog)
        self.serial_port.addItems([' ','COM1','COM2','COM3','COM4','COM5','COM6','COM7','COM8','COM9','COM10'])
        self.serial_port.currentIndexChanged.connect(self.comport_selection)
        self.baudrate.addItems([' ','9600','57600','115200'])
        self.baudrate.currentIndexChanged.connect(self.baude_rate)
        self.Status.setText("No connection")
        self.close_connection.clicked.connect(self.closeport)
        self.Load_Button.setEnabled(False)
        self.startline.clicked.connect(self.gotoline)
        self.actionSet_Connection.triggered.connect(self.show_settings_widget)
        self.closeSettingsButton.clicked.connect(self.hide_settings_widget)
        self.Load_Button.clicked.connect(self.start_sending)
        self.pause_button.setEnabled(False)
        self.pause_button.setText("Pause")
        self.pause_button.clicked.connect(self.toggle_pause)
        self.Xplus_Button.setEnabled(False)
        self.Yplus_Button.setEnabled(False)
        self.Zplus_Button.setEnabled(False)
        self.Xminus_Button.setEnabled(False)
        self.Yminus_Button.setEnabled(False)
        self.Zminus_Button.setEnabled(False)
        # In __init__ or setupUI
        self.jog_sent = False  # flag to prevent multiple sends while holding

        self.Xplus_Button.pressed.connect(lambda: self.jog_pressed("X++"))
        self.Xplus_Button.released.connect(self.jog_released)

        self.Xminus_Button.pressed.connect(lambda: self.jog_pressed("X--"))
        self.Xminus_Button.released.connect(self.jog_released)

        self.Yplus_Button.pressed.connect(lambda: self.jog_pressed("Y++"))
        self.Yplus_Button.released.connect(self.jog_released)

        self.Yminus_Button.pressed.connect(lambda: self.jog_pressed("Y--"))
        self.Yminus_Button.released.connect(self.jog_released)

        self.Zplus_Button.pressed.connect(lambda: self.jog_pressed("Z++"))
        self.Zplus_Button.released.connect(self.jog_released)

        self.Zminus_Button.pressed.connect(lambda: self.jog_pressed("Z--"))
        self.Zminus_Button.released.connect(self.jog_released)






        #self.detect_com_ports()


    def gotoline(self):
        text = self.curentlinenum.text().strip()

        if not text:
            QMessageBox.warning(self, "Input Error", "Line number is empty.")
            return
        try:
            line_number=int(self.curentlinenum.text())
            total_lines=self.listWidget.count()+1
            if line_number>0 and line_number<total_lines:
                self.listWidget.setCurrentRow(line_number -1)
                self.highlightSelectedRow()
            else:
                QMessageBox.warning(self,"out of range",f"please enetr number between 1 and{total_lines-1 }")
        except valueerror:
            QMessageBox.warning(self,"invalid input")
    def show_settings_widget(self):
        self.settingsWidget.show()
        #self.closeSettingsButton.setEnabled(True)
    def hide_settings_widget(self):
        self.settingsWidget.hide() 


    def Reset_values(self):
        self.px = 0
        self.py = 0
        self.pz = 0
        self.newx = 0
        self.newy = 0
        self.newz = 0
        self.dx = 0
        self.dy = 0
        self.dz = 0
        self.xsteps = 0
        self.ysteps = 0
        self.xstepfactor = 1  # Initialize with default value
        self.ystepfactor = 1  # Initialize with default value
        self.pre_frate = 100 # Default feed rate
        self.cur_frate=0
        self.counter=0
        self.CoList=[]
        self.garc=0
        self.r=0
        self.theta1=0
        self.theta2=0
        self.i=0
        self.j=0
        self.cx=0
        self.cy=0
        self.g=0
        self.send=0
        self.ir=0
        self.q=0
        self.glines=[]
        self.selected_line=' '
        self.move_time =0
        self.xy_time=0 
        self.z_time=0
        self.total_work_time=0
        self.hour=0
        self.minutes=0
        self.seconds=0
        self.CoList.clear()
        self.listWidget.clear()
        self.glines.clear()
        self.opengl_widget.clear_shapes()# clear any previous data if needed
        self.progressBar.setValue(0)
        
    def Enable_Jogging(self):
        self.Xplus_Button.setEnabled(True)
        self.Yplus_Button.setEnabled(True)
        self.Zplus_Button.setEnabled(True)
        self.Xminus_Button.setEnabled(True)
        self.Yminus_Button.setEnabled(True)
        self.Zminus_Button.setEnabled(True)
    def Disable_Jogging(self):
        self.Xplus_Button.setEnabled(False)
        self.Yplus_Button.setEnabled(False)
        self.Zplus_Button.setEnabled(False)
        self.Xminus_Button.setEnabled(False)
        self.Yminus_Button.setEnabled(False)
        self.Zminus_Button.setEnabled(False)

    #select comport
    def comport_selection(self):
        self.com=self.serial_port.currentText()
        if self.com !=' ' and self.baude !=' ':
            self.start_button.clicked.connect(self.serialcommunication)
    #select baudrate        
    def baude_rate(self):
        self.baude=self.baudrate.currentText()
        
    def detect_com_ports(self):
        self.serial_port.clear()
        ports = list_ports.comports()
    
        default_port = None
        for port in ports:
            self.serial_port.addItem(port.device)
            # Optional: Detect VSPE or virtual port by description
            if 'VSPE' in port.description or 'Virtual' in port.description:
                default_port = port.device

    # Set default if a virtual port is found
        if default_port:
            index = self.serial_port.findText(default_port)
            if index != -1:
                self.serial_port.setCurrentIndex(index)

    def serialcommunication(self):
        port = self.com
        baudrate = int(self.baude)

        if port == ' ' or baudrate == ' ':
            QMessageBox.critical(self, 'Error', 'Please select a valid COM port and baud rate.')
            return

        try:
            self.ser = serial.Serial(port, int(baudrate), timeout=1)

            if not self.ser.is_open:
                self.ser.open()

            if self.ser.is_open:
                self.Status.setText("Waiting for microcontroller...")
                self.ser.write(b'connect\r\n')

            # Wait for "ready" response
                start_time = time.time()
                self.ready_received = False
                while time.time() - start_time < 10:  # 5-second timeout
                    if self.ser.in_waiting:
                        response = self.ser.readline().decode().strip().lower()
                        if response == "ready":
                            self.ready_received = True

                            break

                if self.ready_received:
                    self.Enable_Jogging()
                    self.Status.setText("Connected")
                    self.start_button.setEnabled(False)
                    self.Load_Button.setEnabled(True)
                    QMessageBox.information(self, 'Info', 'Microcontroller is ready. Serial communication started.')
                else:
                    self.ser.close()
                    self.Status.setText("Not connected")
                    QMessageBox.critical(self, 'Error', 'Microcontroller did not respond with "ready". Connection aborted.')

        except Exception as e:
            QMessageBox.critical(self, 'Error', f'Failed to start serial communication: {e}')

    def closeport(self):
        self.start_button.setEnabled(True)
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.ready_received=False
            self.Disable_Jogging()
            self.FEEDRATE_lineEdit.clear()
            self.Status.setText("not connected")

    def get_feedrate(self):
        try:
            rate = int(self.FEEDRATE_lineEdit.text())
            if rate <= 0:
                return 1
            return rate
        except ValueError:
            return 1

    def send_command(self, command):
        if hasattr(self, 'ser') and self.ready_received:
            feedrate = self.get_feedrate()
            full_command = f"{command}{feedrate}\r\n"
            self.ser.write(full_command.encode())
        else:
            QMessageBox.warning(self, "Warning", "Serial port not connected.")
    def jog_pressed(self, command):
        if self.ready_received and not self.jog_sent:
            feedrate = self.get_feedrate()
            full_command = f"{command}{feedrate}\r\n"
            self.ser.write(full_command.encode())
            self.jog_sent = True 
         # block future sends while holding
        if command.startswith("X"):
            self.jog_axis = "X"
        elif command.startswith("Y"):
            self.jog_axis = "Y"
        elif command.startswith("Z"):
            self.jog_axis = "Z"

    def jog_released(self):
        if self.ready_received and self.jog_axis:
            stop_command = f"{self.jog_axis} stop\r\n"
            self.ser.write(stop_command.encode())

        self.jog_sent = False
        self.jog_axis = None

    
    def start_sending(self):
        print('sending')

        '''if  self.ser.is_open:
            QMessageBox.warning(self, "Error", "Serial port not open.")
            return'''

    # Load glines here if not already loaded
    # For example:
    # with open('your_file.gcode', 'r') as f:
    #     self.glines = f.readlines()

        if not self.glines:
            QMessageBox.warning(self, "Error", "No G-code loaded.")
            return
        else :
            print('threading')
        self.thread = QThread()
        self.sender_thread = GCodeSender(self.ser, self.glines)
        self.sender_thread.progress.connect(self.update_progress)
        self.sender_thread.finished.connect(self.on_send_finished)
        self.sender_thread.lineProcessed.connect(self.line_processed)

        self.sender_thread.error.connect(self.on_send_error)
        self.sender_thread.start()
        self.pause_button.setEnabled(True)
        self.pause_button.setText("Pause")
    def toggle_pause(self):
        if self.sender_thread.paused:
            self.sender_thread.resume()
            self.pause_button.setText("Pause")
        else:
            self.sender_thread.pause()
            self.pause_button.setText("Resume")

    def line_processed(self,value):
        self.listWidget.setCurrentRow(value)
        self.highlightSelectedRow()

    def update_progress(self, value):
        self.progressBar.setValue(value)

    def on_send_finished(self):
        

        self.progressBar.setValue(0)
        self.listWidget.setCurrentRow(0)
        self.highlightSelectedRow()
        QMessageBox.information(self, "Done", "G-code sending completed.")


    def on_send_error(self, message):
        print("Timeout occurred, emitting error signal")
        QMessageBox.critical(self, "Error", message)
               

    def openFileDialog(self):
        options = QFileDialog.Options()
        fileName, _ = QFileDialog.getOpenFileName(self, "Open GCode or Text File", "", "Text Files (*.txt);;GCode Files (*.gcode);;All Files (*)", options=options)
        if fileName:
            with open(fileName, 'r') as file:
                counter=0
                Content = file.read()
                self.Reset_values()
                self.CoList = Content.split("\n")
                self.listWidget.clear()
                self.glines.clear()
                self.opengl_widget.clear_shapes()# clear any previous data if needed
                self.counter = 0
                
                
                self.processing()
                if self.listWidget.count() > 0:
                    self.listWidget.setCurrentRow(0)
                    self.highlightSelectedRow()


    def linearmotion(self, g, x1, y1, z1, x2, y2, z2, frate):
        if g == 90:
            self.newx = x2
            self.newy = y2
            self.newz = z2
        if g == 91:
            self.newx = x1 + x2
            self.newy = y1 + y2
            self.newz = z1 + z2  
        self.dx = self.newx - self.px
        self.dy = self.newy - self.py
        self.dz = self.newz - self.pz
        self.xsteps = int(self.dx * self.xstepfactor)
        self.ysteps = int(self.dy * self.ystepfactor)
        
        if self.newz < 0:
            self.opengl_widget.set_line_parameters((self.px, self.py), (self.newx, self.newy), color=(1.0, 1.0, 1.0))  # White
        if self.newz>0:
            self.opengl_widget.set_line_parameters((self.px, self.py), (self.newx, self.newy), color=(0.0, 0.0, 1.0))  # Blue

        self.px = self.newx
        self.py = self.newy
        self.pz = self.newz
        self.xy_distance = math.sqrt((self.dx) ** 2 + (self.dy) ** 2)
        self.z_distance = abs(self.dz)  # Use abs because Z might be negative (up/down)
        

        # === Time calculation ===
        if self.xy_distance > 0 and frate > 0 :
            self.xy_time = (self.xy_distance / frate) * 60 
        if self.z_distance  > 0 and frate > 0 :
            self.z_time  = (self.z_distance  / frate) * 60  
        self.move_time = self.xy_time + self.z_time
        self.total_work_time += self.move_time
    

    def  circularmotion(self,arc,g,x1,y1,z1,x2,y2,z2,i,j,frate):
        if g==90:
            self.newx=x2
            self.newy=y2
            self.newz=z2
            self.cx=i
            self.cy=j       
        if g==91 :
            self.newx=x1+x2
            self.newy=y1+y2
            self.newz=z1+z2
            self.cx=x1+i
            self.cy=y1+j   
        self.r=math.sqrt((x1-self.cx)**2+(y1-self.cy)**2)
        # calculate theta1
        if (x1-self.cx)==0 :
            if(y1-self.cy)>0:
                self.theta1=math.pi/2
            if (y1-self.cy)<0:
                self.theta1=math.pi*3/2
        else:
            d=((self.py-self.cy)/(self.px-self.cx))
            if d==0:
                if(x1-self.cx)>0:
                    self.theta1=math.pi*2
                if(x1-self.cx)<0:
                    self.theta1=math.pi
            if d!=0:
                if d>0:
                    if(y1-self.cy)>0 and (x1-self.cx)>0:
                        self.theta2=math.atan(d)
                    else:
                        self.theta2=math.pi+math.atan(d)
                else:
                    if (y1-self.cy)>0:
                        
                        self.theta1=math.atan(d)+math.pi
                    else :
                        self.theta1=math.atan(d)+math.pi*2
        #calculte theta2
        if (self.newx-self.cx)==0 :
            if(self.newy-self.cy)>0:
                self.theta2=math.pi/2
            if (self.newy-self.cy)<0:
                self.theta2=math.pi*3/2
        else:
            d1=((self.newy-self.cy)/(self.newx-self.cx))
            
            if d1==0:
                if(self.newx-self.cx)>0:
                    self.theta2=math.pi*2
                if(self.newx-self.cx)<0:
                    self.theta2=math.pi
            if d1!=0:
                if d1>0:
                    if(self.newy-self.cy)>0 and (self.newx-self.cx)>0:
                        self.theta2=math.atan(d1)
                  
                    else:
                        self.theta2=math.pi+math.atan(d1)
                else:
                    if (self.newy-self.cy)>0:
                        self.theta2=math.atan(d1)+math.pi
                    else :
                        
                        self.theta2=math.atan(d1)+math.pi*2
        
        if arc ==3 :
            theta = (self.theta2 - self.theta1 + math.pi*2) % (2*math.pi)
        #G2 (CW):
        if arc==2:
            theta = (self.theta1 - self.theta2 + math.pi*2) % (2*math.pi)
        arc_length = self.r * theta
        
        if arc_length > 0 and frate > 0:
            self.arc_time = (arc_length / frate) * 60  # time in seconds
        else:
            self.arc_time = 0
        self.total_work_time += self.arc_time
        
        # Determine arc color based on newz
        if self.newz < 0:
            arc_color = (1.0, 1.0, 1.0)  # White
        else:
            arc_color = (0.0, 0.0, 1.0)  # Blue

        self.opengl_widget.set_arc_parameters(arc, x1, y1, self.cx, self.cy, self.r, self.theta1, self.theta2, color=arc_color)

        #self.opengl_widget.set_arc_parameters(arc,x1,y1,self.cx, self.cy, self.r, self.theta1, self.theta2)
        self.px=self.newx
        self.py=self.newy
        self.pz=self.newz 

    def update_time_display(self):
        self.hour = int(self.total_work_time // 3600)
        self.minutes = int((self.total_work_time % 3600) // 60)
        self.seconds = int(self.total_work_time % 60)

        # Format time string as HH:MM:SS
        time_str = f"{self.hour:02}:{self.minutes:02}:{self.seconds:02}"
    
        # Set to lineEdit
        self.lineEdit_time.setText(time_str)

    def parsing(self,ir): 
        pattern_m = re.compile(r'[Mm](-?\d+(\.\d+)?)')
        pattern_g = re.compile(r'[Gg](-?\d+(\.\d+)?)')
        pattern_x = re.compile(r'[Xx](-?\d+(\.\d+)?)')
        pattern_y = re.compile(r'[Yy](-?\d+(\.\d+)?)')
        pattern_z = re.compile(r'[Zz](-?\d+(\.\d+)?)')
        pattern_i = re.compile(r'[Ii](-?\d+(\.\d+)?)')
        pattern_j = re.compile(r'[Jj](-?\d+(\.\d+)?)')
        pattern_f = re.compile(r'[Ff](-?\d+(\.\d+)?)')
        pattern_n = re.compile(r'[Nn](-?\d+(\.\d+)?)')
        pattern_p = re.compile(r'[Pp](-?\d+(\.\d+)?)')


        matches_m = pattern_m.findall(self.CoList[ir])
        matches_g = pattern_g.findall(self.CoList[ir])
        matches_x = pattern_x.findall(self.CoList[ir])
        matches_y = pattern_y.findall(self.CoList[ir])
        matches_z = pattern_z.findall(self.CoList[ir])
        matches_i = pattern_i.findall(self.CoList[ir])
        matches_j = pattern_j.findall(self.CoList[ir])
        matches_f = pattern_f.findall(self.CoList[ir])
        matches_n = pattern_n.findall(self.CoList[ir])
        matches_p = pattern_p.findall(self.CoList[ir])

        #global g_values,x_values,y_values,z_values,i_values,j_values,m_values,f_values
        self.m_values = [float(match[0]) for match in matches_m]
        #  global px,py,pz,newx,newy,newz,dx,dy,dz,xsteps,ysteps,xstepfactor,ystepfactor,frate,i,j
        
        self.g_values = [float(match[0]) for match in matches_g]
        self.x_values = [float(match[0]) for match in matches_x]
        self.y_values = [float(match[0]) for match in matches_y]
        self.z_values = [float(match[0]) for match in matches_z]
        self.i_values = [float(match[0]) for match in matches_i]
        self.j_values = [float(match[0]) for match in matches_j]
        self.f_values = [float(match[0]) for match in matches_f]
        self.n_values = [float(match[0]) for match in matches_n]
        self.p_values = [float(match[0]) for match in matches_p]

    def processing(self):       
        for j in range(0,len(self.CoList)):
            self.parsing(j)
            #if self.g_values or self.m_values or self.n_values:
            x=self.CoList[j].upper().strip()
            if x:
                self.glines.append(x)
                self.listWidget.addItem(x)
                self.counter +=1
                self.start_line.addItem(str(self.counter))
                #self.baudrate.addItems([' ','9600','57600','115200'])
                self.number=str(self.counter)
                self.linenum.setText(self.number)   
            if self.f_values:
                self.cur_frate=self.f_values[0]
                self.pre_frate=self.cur_frate
            else:
                self.cur_frate=self.pre_frate
            if self.g_values:
                if self.g_values[0] == 4:  # G4 dwell comman
                    if self.p_values:
                        dwell_seconds = self.p_values[0] # assuming P is in milliseconds
                        self.total_work_time += int(dwell_seconds)
                if self.g_values[0] == 90:
                    self.g = 90
                if self.g_values[0] == 91:
                    self.g = 91
                if self.g_values[0] == 0:    
                    if self.g==90:
                        self.newx = self.x_values[0] if self.x_values else self.px
                        self.newy = self.y_values[0] if self.y_values else self.py
                        self.newz = self.z_values[0] if self.z_values else self.pz
                        self.linearmotion(90, self.px, self.py, self.pz, self.newx, self.newy, self.newz, self.cur_frate)
                    if self.g==91:
                        self.newx = self.x_values[0] if self.x_values else 0
                        self.newy = self.y_values[0] if self.y_values else 0
                        self.newz = self.z_values[0] if self.z_values else 0
                        self.linearmotion(91, self.px, self.py, self.pz, self.newx, self.newy, self.newz, self.cur_frate)

                if self.g_values[0] == 1:
                    
                    if self.g==90:
                        self.newx = self.x_values[0] if self.x_values else self.px
                        self.newy = self.y_values[0] if self.y_values else self.py
                        self.newz = self.z_values[0] if self.z_values else self.pz
                        self.linearmotion(90, self.px, self.py, self.pz, self.newx, self.newy, self.newz, self.cur_frate)
                    if self.g==91:
                        self.newx = self.x_values[0] if self.x_values else 0
                        self.newy = self.y_values[0] if self.y_values else 0
                        self.newz = self.z_values[0] if self.z_values else 0
                        self.linearmotion(91, self.px, self.py, self.pz, self.newx, self.newy, self.newz, self.cur_frate)

                if self.g_values[0] ==2 or self.g_values[0] ==3:
                    self.garc=self.g_values[0]
                    self.i = self.i_values[0] #if self.i_values else 0
                    self.j = self.j_values[0] #if self.j_values else 0
                    if self.g==90:
                        self.newx = self.x_values[0] if self.x_values else self.px
                        self.newy = self.y_values[0] if self.y_values else self.py
                        self.newz = self.z_values[0] if self.z_values else self.pz
                        self.circularmotion(self.garc ,self.g, self.px, self.py,self.pz,  self.newx, self.newy,self.newz, self.i,self.j,self.cur_frate)
                    if self.g==91:
                        self.newx = self.x_values[0] if self.x_values else 0
                        self.newy = self.y_values[0] if self.y_values else 0
                        self.newz = self.z_values[0] if self.z_values else 0
                        self.circularmotion(self.garc ,self.g, self.px, self.py,self.pz,  self.newx, self.newy,self.newz ,self.i,self.j,self.cur_frate)

            if self.m_values :
                if self.m_values[0]== 5 or self.m_values[0]== 30 :
                    self.update_time_display()
                    

        

    def highlightSelectedRow(self):
        selected_row = self.listWidget.currentRow()
        if selected_row < 0:
            return

        for i in range(self.listWidget.count()):
            item = self.listWidget.item(i)
            if i == selected_row:
                item.setBackground(QColor('blue'))
                item.setForeground(QColor('white'))
            else:
                item.setBackground(QColor('white'))
                item.setForeground(QColor('black'))

    # Update line and number
        selected_item = self.listWidget.currentItem()
        if selected_item:
            self.currentline.setText(selected_item.text())
            self.curentlinenum.setText(str(selected_row+1))


    def on_item_clicked(self, item):
        self.highlightSelectedRow()

    def closeEvent(self,event):
        reply = QMessageBox.question(self, 'Message', 
            "Are you sure you want to quit?", QMessageBox.Yes | 
            QMessageBox.No, QMessageBox.No)

        if reply == QMessageBox.Yes:
            if self.ser.is_open:
                self.ser.close()
           
        
            
            QApplication.quit()
        if reply == QMessageBox.No:
            event.ignore()
           

        
def main():
    app = QApplication(sys.argv)
    window = MyMainWindow()
    window.show()
    window.showMaximized()

    sys.exit(app.exec_())

if __name__ == "__main__":
    main()

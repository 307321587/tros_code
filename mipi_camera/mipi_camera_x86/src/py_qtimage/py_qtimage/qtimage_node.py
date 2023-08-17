from .Ui_new import Ui_MainWindow

import rclpy
from rclpy.node import Node
from qt_image.msg import QtImage
from std_msgs.msg import String

import cv2
import numpy as np
from PyQt5.QtGui import QPixmap,QImage
from PyQt5.QtCore import pyqtSignal, QObject
from PyQt5.QtWidgets import QMainWindow
from PyQt5.QtWidgets import QApplication,QWidget,QTextEdit,QVBoxLayout,QPushButton
import threading
import sys

class MainWindow(QMainWindow,Ui_MainWindow):
    def __init__(self):
        QMainWindow.__init__(self)
        Ui_MainWindow.__init__(self)
        # self.receiver = Subscriber()

        #在界面主线程中开启守护线程启动节点
        self.node = None
        t_ = threading.Thread(target=self.thread_ros, args=())
        t_.setDaemon(True)                        # 主线程崩掉，守护线程可以自己shutdown
        t_.start()

    def thread_ros(self):
        rclpy.init(args=None)                     # 初始化python接口函数
        self.node = Subscriber()
        rclpy.spin(self.node)
        self.node.destroy_node()
        rclpy.shutdown()

class Subscriber(Node,QObject):
    def __init__(self):
        print(threading.current_thread())
        super().__init__('subscriber')
        self.img_subscription = self.create_subscription(QtImage,"qt_image",self.image_callback,100)        #image
        # self.img_subscription = self.create_subscription(
        #             String,
        #             'topic',
        #             self.image_callback,
        #             10)

        self.img_subscription  # prevent unused variable warnin
        
        self.image_show=pyqtSignal(QPixmap)

    def image_callback(self, msg):
        print('image process start...')
        # array=np.array(bytearray(msg.serialize_image), dtype='uint8')
        # input = cv2.imdecode(array, cv2.IMREAD_COLOR)
        # h,w,c=input.shape
        # image=QImage(input.data,w,h,QImage.Format_RGB888)
        # temp_pix=QPixmap.fromImage(image)
        # pix=temp_pix.scaled(show_width,show_height)
        # self.image_show.emit(temp_pix)
        print('image process completed!')

def main(args=None):
    print(threading.current_thread())
    
    rclpy.init(args=None)                     # 初始化python接口函数
    node = Subscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

    # app=QApplication(sys.argv)
    # hmi_ws=MainWindow()
    # hmi_ws.show()
    # sys.exit(app.exec_())
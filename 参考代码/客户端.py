import struct
import cv2
import socket
import time
import math


MSG_VIDEO = 0x01
MSG_STATUS = 0x02


class MultiVideoSender:
    def __init__(self, target_ip, port):
        self.target_ip = target_ip
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 2 * 1024 * 1024)
        self.MAX_UDP_SIZE = 60000
        self.quality = 50

    # -----------------------------
    # 发送视频帧 (协议保持不变: 头+CamID+图片数据)
    # -----------------------------
    def send_frame(self, camera_id, frame):
        if frame is None:
            return

        # 压缩尺寸以保证流畅传输
        frame = cv2.resize(frame, (640, 360))
        _, encoded = cv2.imencode(
            '.jpg', frame,
            [cv2.IMWRITE_JPEG_QUALITY, self.quality]
        )
        jpeg_data = encoded.tobytes()

        # 头部：消息类型(1B) + 相机ID(1B)
        header = struct.pack('!BB', MSG_VIDEO, camera_id)
        packet = header + jpeg_data

        if len(packet) < self.MAX_UDP_SIZE:
            self.sock.sendto(packet, (self.target_ip, self.port))

    # -----------------------------
    # 发送态势数据 (关键修改)
    # -----------------------------
    def send_status(self, camera_id, target_id, azimuth, elevation, distance):
        """
        新增 target_id 参数
        格式: !BBIfff
        B: MsgType (1字节)
        B: CamID (1字节)
        I: TargetID (4字节, unsigned int) -> 对应接收端的 struct.unpack('!I...')
        f: Azimuth (4字节 float)
        f: Elevation (4字节 float)
        f: Distance (4字节 float)
        总长度: 1 + 1 + 4 + 4 + 4 + 4 = 18 字节
        """
        packet = struct.pack(
            '!BBIfff',
            MSG_STATUS,
            camera_id,
            int(target_id),  # 确保转为整数
            azimuth,
            elevation,
            distance
        )
        self.sock.sendto(packet, (self.target_ip, self.port))


# -----------------------------
# 模拟推送
# -----------------------------
if __name__ == "__main__":
    # 请根据实际情况修改 IP，如果是本机测试用 127.0.0.1
    sender = MultiVideoSender('127.0.0.1', 9999)

    # 打开摄像头或视频文件 (请确保路径正确)
    cap0 = cv2.VideoCapture(0)  # 也可以换成视频文件路径
    # 如果没有这么多视频文件，可以都改成 0 或者是同一个文件路径测试
    cap1 = cv2.VideoCapture(r"G:\研究生\项目\空中机动小目标\video\IMG_2109.MP4")
    cap2 = cv2.VideoCapture(r"G:\研究生\项目\空中机动小目标\920\video\IMG_3174.MP4")
    cap3 = cv2.VideoCapture(r"G:\研究生\项目\空中机动小目标\920\111111111.MP4")

    print("开始推送 4 路视频 + 多目标态势数据...")

    t = 0.0

    while True:
        ret0, frame0 = cap0.read()
        ret1, frame1 = cap1.read()
        ret2, frame2 = cap2.read()
        ret3, frame3 = cap3.read()

        # 如果视频读完了，稍微处理一下（这里简单跳出，实际可以循环播放）
        if not (ret0 or ret1 or ret2 or ret3):  # 改成 or 防止一个读完就全停
            print("视频读取结束")
            break

        # 容错处理：如果某个视频读不到了，给个黑帧，防止报错
        if frame0 is None: frame0 = cv2.resize(frame1 if frame1 is not None else frame0, (640, 360))  # 仅示例

        # ---------------- CAM 0 (模拟双目标) ----------------
        if ret0:
            az0 = (t * 20) % 360
            el0 = 10 * math.sin(t)
            dist0 = 400 + 30 * math.cos(t * 0.5)

            sender.send_frame(0, frame0)

            # 发送目标 A (ID: 101)
            sender.send_status(camera_id=0, target_id=101, azimuth=az0, elevation=el0, distance=dist0)

            # 发送目标 B (ID: 102) - 同一个相机，不同的目标ID，不同的数据
            sender.send_status(camera_id=0, target_id=102, azimuth=az0 * 1.1, elevation=el0 + 5, distance=dist0 + 60)

        # ---------------- CAM 1 (单目标) ----------------
        if ret1:
            az1 = (t * 15 + 90) % 360
            el1 = 8 * math.cos(t)
            dist1 = 300 + 20 * math.sin(t * 0.3)

            sender.send_frame(1, frame1)

            # 发送目标 (ID: 201)
            sender.send_status(camera_id=1, target_id=201, azimuth=az1, elevation=el1, distance=dist1)

        # ---------------- CAM 2 (单目标) ----------------
        if ret2:
            az2 = (t * 10 + 180) % 360
            el2 = 6 * math.sin(t * 0.8)
            dist2 = 200 + 40 * math.cos(t * 0.4)

            sender.send_frame(2, frame2)

            # 发送目标 (ID: 301)
            sender.send_status(camera_id=2, target_id=301, azimuth=az2, elevation=el2, distance=dist2)

        # ---------------- CAM 3 (单目标) ----------------
        if ret3:
            az3 = (t * 25 + 270) % 360
            el3 = 12 * math.cos(t * 0.6)
            dist3 = 100 + 25 * math.sin(t * 0.2)

            sender.send_frame(3, frame3)

            # 发送目标 (ID: 401)
            sender.send_status(camera_id=3, target_id=401, azimuth=az3, elevation=el3, distance=dist3)

        t += 0.1
        # 控制发送频率，大约 20fps
        time.sleep(0.05)

    cap0.release()
    cap1.release()
    cap2.release()
    cap3.release()
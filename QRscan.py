import cv2

class QrReader:
    def __init__(self):
        self.cam = cv2.VideoCapture(0)
        self.qcd = cv2.QRCodeDetector()

    def read_qr(self):
        ret, frame = self.cam.read()

        if not ret:
            return False

        cv2.imshow('frame', frame)

        ret_qr, decoded_info, points, _ = self.qcd.detectAndDecodeMulti(frame)
        if ret_qr:
            for s, p in zip(decoded_info, points):
                if s:
                    return True

    def clean(self):
        cv2.destroyAllWindows()
        self.cam.release()
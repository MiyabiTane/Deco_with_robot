import cv2
import argparse

class MosaicClickedPos:
    def __init__(self, input_img):
        self.img = input_img
        self.clicked_pos = []

    def get_clicked_pos(self):
        def onMouse(event, x, y, flags, params):
            if event == cv2.EVENT_LBUTTONDOWN:
                print("clicked x, y:", x, y)
                self.clicked_pos.append((x, y))

        print("Please click the position you want to blur")
        print("If finish, press any key")
        cv2.namedWindow('window', cv2.WINDOW_NORMAL)
        cv2.imshow('window', self.img)
        cv2.setMouseCallback('window', onMouse)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    
    def make_mosaic(self, x, y, width=30, height=30, ratio=0.1):
        mosaic_img = self.img.copy()
        output_img = self.img.copy()
        mosaic_img = mosaic_img[y:y + height, x:x + width]
        small_img = cv2.resize(mosaic_img, None, fx=ratio, fy=ratio, interpolation=cv2.INTER_NEAREST)
        mosaic_img = cv2.resize(small_img, mosaic_img.shape[:2][::-1], interpolation=cv2.INTER_NEAREST)
        output_img[y:y + height, x:x + width] = mosaic_img
        self.img = output_img
    
    def mosaic_main(self):
        while self.clicked_pos:
            x, y = self.clicked_pos.pop(0)
            self.make_mosaic(x, y)
        return self.img

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True)
    parser.add_argument("--output", default="output.png")
    args = parser.parse_args()

    input_img = cv2.imread(args.input)
    mosaic_clicked_pos = MosaicClickedPos(input_img)
    mosaic_clicked_pos.get_clicked_pos()
    output_img = mosaic_clicked_pos.mosaic_main()
    while True:
        print("If finish checking, please press any key")
        cv2.namedWindow('window', cv2.WINDOW_NORMAL)
        cv2.imshow('window', output_img)
        # cv2.waitKey(5000)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        input_text = raw_input("continue? [Y/n]: ")
        if input_text != 'Y':
            cv2.imwrite(args.output, output_img)
            print("output img as {}".format(args.output))
            break
        mosaic_clicked_pos.get_clicked_pos()
        output_img = mosaic_clicked_pos.mosaic_main()

main()

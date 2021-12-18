import numpy as np
import cv2

#データ点数
data_num = 18;

#データ点格納用
points = np.zeros([2,data_num,2],dtype = int) #格納配列
pt = np.array([0,0]) #要素地点管理

#マウスイベント処理(leftimg)
def mouse_event_l(event, x, y, flags, param):
    #配列外参照回避
    if pt[0] > (data_num-1):
        return
    #クリック地点を配列に格納
    if event == cv2.EVENT_LBUTTONUP:
        points[0][pt[0]] = [x,y] #格納
        cv2.circle(window_l, (x,y), 5, (255,0,0), -1)
        pt[0] += 1 #要素地点を1つ増やす

#マウスイベント処理(rightimg)
def mouse_event_r(event, x, y, flags, param):
    #配列外参照回避
    if pt[1] > (data_num-1):
        return
    #クリック地点を配列に格納
    if event == cv2.EVENT_LBUTTONUP:
        points[1][pt[1]] = [x,y]
        cv2.circle(window_r, (x,y), 5, (0,0,255), -1)
        pt[1] += 1 #要素地点を1つ増やす

#画像の読み込み
window_l = cv2.imread("kiyooka/frame000142.jpg", 1) #leftimg
window_r = cv2.imread("kiyooka/frame000118.jpg", 1) #rightimg

#ウィンドウ生成
cv2.namedWindow("window_l", cv2.WINDOW_KEEPRATIO) #leftimg
cv2.namedWindow("window_r", cv2.WINDOW_KEEPRATIO) #rightimg

#マウスイベント時に関数mouse_eventの処理を行う
cv2.setMouseCallback("window_l", mouse_event_l) #leftimg
cv2.setMouseCallback("window_r", mouse_event_r) #rightimg

#「q」が押されるまでループ
while True:
    #画像の表示
    cv2.imshow("window_l", window_l) #leftimg
    cv2.imshow("window_r", window_r) #rightimg
    print(points)

    #キー入力
    if cv2.waitKey(1) & 0xFF == ord("a"):
        break

cv2.destroyAllWindows()

print(points)

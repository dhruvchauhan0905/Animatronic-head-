import cv2
import mediapipe as mp
import serial
import time
import math

arduino = serial.Serial('COM9', 9600)
time.sleep(2)

mp_face_mesh = mp.solutions.face_mesh
face_mesh = mp_face_mesh.FaceMesh(refine_landmarks=True)

cap = cv2.VideoCapture(0)

ROBOT_EYE_X_CENTER = 330
ROBOT_EYE_Y_CENTER = 350
ROBOT_NECK6_CENTER = 90
ROBOT_NECK7_CENTER = 90
ROBOT_NECK8_CENTER = 90

smoothX = ROBOT_EYE_X_CENTER
smoothY = ROBOT_EYE_Y_CENTER
L2 = L3 = R4 = R5 = 330

targetNeck6 = ROBOT_NECK6_CENTER
targetNeck7 = ROBOT_NECK7_CENTER
targetNeck8 = ROBOT_NECK8_CENTER

yawHistory = []
verticalHistory = []
tiltHistory = []

lastLeftX = None
lastRightX = None

baseTop = 0
baseBottom = 0
baseTilt = 0
baseYaw = 0
baseEyeRelX = 0
baseEyeRelY = 0

neckCenterLocked = False
yawCenterLocked = False

leftEyeStateOpen = True
rightEyeStateOpen = True

def map_range(value, in_min, in_max, out_min, out_max):
    value = max(min(value, in_max), in_min)
    return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

def distance(a, b):
    return math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2)

def easeBlend(t):
    t = max(0.0,min(1.0,t))
    return (1 - math.cos(math.pi*t))/2

def motorColor(val, neutral):
    diff = abs(val-neutral)
    if diff < 8:
        return (0,255,0)
    elif diff < 30:
        return (0,255,255)
    else:
        return (0,0,255)

def drawMotorValues(frame, motors):
    cv2.rectangle(frame,(910,10),(1270,380),(25,25,25),-1)
    cv2.putText(frame,"LIVE MOTOR VALUES",(980,35),
                cv2.FONT_HERSHEY_SIMPLEX,0.75,(255,255,255),2)

    y = 70
    for name,val,neutral in motors:
        color = motorColor(val,neutral)
        cv2.putText(frame,f"{name}: {int(val)}",(930,y),
                    cv2.FONT_HERSHEY_SIMPLEX,0.62,color,2)
        y += 33

print("Sit comfortably in front of camera...")

# ================= CALIBRATION =================
start_time = time.time()
confirmed = False
tentative_capture = False
temp_data = {}

while not confirmed:
    ret, frame = cap.read()
    frame = cv2.flip(frame,1)
    frame = cv2.resize(frame,(1280,720))
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = face_mesh.process(rgb)

    h,w,_ = frame.shape

    if results.multi_face_landmarks:
        lm = results.multi_face_landmarks[0].landmark
        xs = [int(p.x*w) for p in lm]
        ys = [int(p.y*h) for p in lm]
        x1,x2 = min(xs), max(xs)
        y1,y2 = min(ys), max(ys)

        cv2.rectangle(frame,(x1,y1),(x2,y2),(0,255,0),2)

        if not tentative_capture:
            if time.time()-start_time > 10:
                cv2.putText(frame,"SET THIS AS USER NEUTRAL? Press Y / N",(20,50),
                            cv2.FONT_HERSHEY_SIMPLEX,0.8,(0,255,255),2)
        else:
            cv2.putText(frame,"CONFIRM USER REFERENCE ZERO? Press Y / N",(20,50),
                        cv2.FONT_HERSHEY_SIMPLEX,0.8,(0,255,255),2)

    cv2.imshow("Calibration",frame)
    key = cv2.waitKey(1) & 0xFF

    if key == ord('y') and results.multi_face_landmarks and not tentative_capture:
        lm = results.multi_face_landmarks[0].landmark
        forehead = lm[10]; nose = lm[1]; chin = lm[152]
        leftFace = lm[234]; rightFace = lm[454]
        irisL = lm[468]; irisR = lm[473]

        leftRatioX = (irisL.x-lm[33].x)/(lm[133].x-lm[33].x)
        rightRatioX = (irisR.x-lm[362].x)/(lm[263].x-lm[362].x)
        leftRatioY = (irisL.y-lm[159].y)/(lm[145].y-lm[159].y)
        rightRatioY = (irisR.y-lm[386].y)/(lm[374].y-lm[386].y)

        temp_data["baseEyeRelX"]=(leftRatioX+rightRatioX)/2
        temp_data["baseEyeRelY"]=(leftRatioY+rightRatioY)/2

        faceWidth = distance(leftFace,rightFace)
        temp_data["baseTop"]=distance(forehead,nose)/faceWidth
        temp_data["baseBottom"]=distance(nose,chin)/faceWidth
        temp_data["baseTilt"]=math.degrees(math.atan2((rightFace.y-leftFace.y),(rightFace.x-leftFace.x)))

        dL = abs(nose.x-leftFace.x)
        dR = abs(rightFace.x-nose.x)
        temp_data["baseYaw"]=(dR-dL)/(dR+dL)

        tentative_capture=True

    elif key == ord('y') and tentative_capture:
        baseTop=temp_data["baseTop"]
        baseBottom=temp_data["baseBottom"]
        baseTilt=temp_data["baseTilt"]
        baseYaw=temp_data["baseYaw"]
        baseEyeRelX=temp_data["baseEyeRelX"]
        baseEyeRelY=temp_data["baseEyeRelY"]
        confirmed=True

    elif key == ord('n'):
        tentative_capture=False
        start_time=time.time()-5

cv2.destroyWindow("Calibration")

# ================= MAIN LOOP =================
while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame = cv2.flip(frame,1)
    frame = cv2.resize(frame,(1280,720))
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = face_mesh.process(rgb)

    if results.multi_face_landmarks:
        lm = results.multi_face_landmarks[0].landmark
        h,w,_ = frame.shape

        leftOpen = distance(lm[159],lm[145])
        rightOpen = distance(lm[386],lm[374])

        if leftOpen > 0.0132: leftEyeStateOpen = True
        elif leftOpen < 0.0108: leftEyeStateOpen = False

        if rightOpen > 0.0132: rightEyeStateOpen = True
        elif rightOpen < 0.0108: rightEyeStateOpen = False

        irisL = lm[468]; irisR = lm[473]
        cv2.circle(frame,(int(irisL.x*w),int(irisL.y*h)),2,(0,255,0),-1)
        cv2.circle(frame,(int(irisR.x*w),int(irisR.y*h)),2,(255,0,0),-1)

        leftRatioX = (irisL.x-lm[33].x)/(lm[133].x-lm[33].x)
        rightRatioX = (irisR.x-lm[362].x)/(lm[263].x-lm[362].x)
        avgRelX = (leftRatioX+rightRatioX)/2

        leftRatioY = (irisL.y-lm[159].y)/(lm[145].y-lm[159].y)
        rightRatioY = (irisR.y-lm[386].y)/(lm[374].y-lm[386].y)
        avgRelY = (leftRatioY+rightRatioY)/2

        forehead = lm[10]; noseMain = lm[1]; chin = lm[152]
        leftFace = lm[234]; rightFace = lm[454]

        faceWidth = distance(leftFace,rightFace)
        topNorm = distance(forehead,noseMain)/faceWidth
        bottomNorm = distance(noseMain,chin)/faceWidth
        tiltAngle = math.degrees(math.atan2((rightFace.y-leftFace.y),(rightFace.x-leftFace.x)))

        dLtemp = abs(noseMain.x-leftFace.x)
        dRtemp = abs(rightFace.x-noseMain.x)
        yawTemp = ((dRtemp-dLtemp)/(dRtemp+dLtemp)) - baseYaw

        yawDamp = max(0.35,1-abs(yawTemp)*2.0)

        rawVertical = ((baseTop-topNorm)+(bottomNorm-baseBottom))*950
        rawTilt = (tiltAngle-baseTilt)*3.0*yawDamp

        verticalHistory.append(rawVertical)
        tiltHistory.append(rawTilt)

        if len(verticalHistory)>10: verticalHistory.pop(0)
        if len(tiltHistory)>10: tiltHistory.pop(0)

        verticalRatio = sum(verticalHistory)/len(verticalHistory)
        horizontalTilt = sum(tiltHistory)/len(tiltHistory)

        neckError = abs(verticalRatio)+abs(horizontalTilt)

        if neckCenterLocked:
            if neckError > 13: neckCenterLocked = False
        else:
            if neckError < 5: neckCenterLocked = True

        if neckCenterLocked:
            targetNeck6 = ROBOT_NECK6_CENTER
            targetNeck7 = ROBOT_NECK7_CENTER
            cv2.putText(frame,"NECK CENTER LOCKED",(20,80),
                        cv2.FONT_HERSHEY_SIMPLEX,0.75,(0,255,0),2)
        else:
            targetNeck6 = max(min(ROBOT_NECK6_CENTER + verticalRatio + horizontalTilt,175),5)
            targetNeck7 = max(min(ROBOT_NECK7_CENTER - verticalRatio + horizontalTilt,175),5)

        if lastLeftX is None:
            lastLeftX = leftFace.x
            lastRightX = rightFace.x

        leftX = leftFace.x*0.6 + lastLeftX*0.4
        rightX = rightFace.x*0.6 + lastRightX*0.4
        lastLeftX = leftX
        lastRightX = rightX

        dL = abs(lm[1].x-leftX)
        dR = abs(rightX-lm[1].x)
        yawRatio = (dR-dL)/(dR+dL)
        yawRelative = yawRatio-baseYaw

        yawHistory.append(yawRelative)
        if len(yawHistory)>10: yawHistory.pop(0)

        avgYaw = sum(yawHistory)/len(yawHistory)

        if yawCenterLocked:
            if abs(avgYaw) > 0.09: yawCenterLocked=False
        else:
            if abs(avgYaw) < 0.04: yawCenterLocked=True

        if yawCenterLocked:
            targetNeck8 = ROBOT_NECK8_CENTER
            cv2.putText(frame,"YAW CENTER LOCKED",(20,45),
                        cv2.FONT_HERSHEY_SIMPLEX,0.75,(0,255,0),2)
        else:
            targetNeck8 = map_range(avgYaw,-0.30,0.30,3,175)

        effectiveY = avgRelY
        if avgYaw < -0.16:
            t = min(abs(avgYaw+0.16)/0.10,1.0)
            effectiveY = avgRelY*(1-0.35*t)+leftRatioY*(0.35*t)
        elif avgYaw > 0.16:
            t = min(abs(avgYaw-0.16)/0.10,1.0)
            effectiveY = avgRelY*(1-0.35*t)+rightRatioY*(0.35*t)

        deltaX = (avgRelX-baseEyeRelX)*100
        deltaY = (effectiveY-baseEyeRelY)*100

        targetX = max(min(ROBOT_EYE_X_CENTER + deltaX*8,465),195)
        targetY = max(min(ROBOT_EYE_Y_CENTER - deltaY*6,445),260)

        smoothX += (targetX-smoothX)*0.90
        smoothY += (targetY-smoothY)*0.67

        blendLeft=0.0; blendRight=0.0
        if targetNeck8 < 48: blendLeft = easeBlend((48-targetNeck8)/16.0)
        if targetNeck8 > 132: blendRight = easeBlend((targetNeck8-132)/16.0)

        effRightState = rightEyeStateOpen
        effLeftState = leftEyeStateOpen

        if targetNeck8 < 48 and blendLeft > 0.5:
            effRightState = leftEyeStateOpen
        if targetNeck8 > 132 and blendRight > 0.5:
            effLeftState = rightEyeStateOpen

        targetR2,targetR3 = (477,193) if effRightState else (330,330)
        targetL4,targetL5 = (193,477) if effLeftState else (330,330)

        L2 += (targetR2-L2)*0.39
        L3 += (targetR3-L3)*0.39
        R4 += (targetL4-R4)*0.39
        R5 += (targetL5-R5)*0.39

        sendData = f"{int(smoothX)},{int(smoothY)},{int(L2)},{int(L3)},{int(R4)},{int(R5)},{int(targetNeck6)},{int(targetNeck7)},{int(targetNeck8)}\n"
        arduino.write(sendData.encode())

        motors = [
            ("CH0 EyeX", smoothX,330),
            ("CH1 EyeY", smoothY,350),
            ("CH2 LidR1", L2,330),
            ("CH3 LidR2", L3,330),
            ("CH4 LidL1", R4,330),
            ("CH5 LidL2", R5,330),
            ("CH6 NeckA", targetNeck6,90),
            ("CH7 NeckB", targetNeck7,90),
            ("CH8 NeckYaw", targetNeck8,90)
        ]

        drawMotorValues(frame,motors)

    cv2.imshow("Final Realistic System V4 Master", frame)

    if cv2.waitKey(1)==27:
        break

cap.release()
cv2.destroyAllWindows()
arduino.close()
